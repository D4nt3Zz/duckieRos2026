#!/usr/bin/env python3
# =============================================================================
# mission_controller_node.py
# Duckie Day Puerto Montt 2026
#
# PROPOSITO:
#   Orquestador central del sistema. Es el UNICO nodo con autoridad para
#   publicar en /{veh}/conduccion/estado (habilitar/deshabilitar conduccion).
#
#   Recibe eventos de todos los subsistemas mediante un unico topico de entrada
#   (/{veh}/conduccion/evento) y de seguridad (/{veh}/conduccion/block_seguridad).
#   Decide el estado global y notifica al lane_follow y al led_emitter.
#
# ESTADOS:
#   DRIVING      -> Conduccion PID normal
#   STOPPING     -> Detenido ante linea roja (duracion configurada en YAML)
#   MANEUVERING  -> Ejecutando instruccion de giro/recto
#   BLOCKED      -> Parada de emergencia (ToF o vehiculo detectado)
#   QR_ACTION    -> Ejecutando mision de codigo QR
#   RETURN_HOME  -> Buscando linea azul de retorno
#   FINALIZADO   -> Robot detenido permanentemente (linea azul encontrada)
#
# TOPICOS SUSCRITOS:
#   /{veh}/conduccion/evento          String (JSON) — eventos de lane_follow, apriltag, qr
#   /{veh}/mision/instrucciones       String (JSON) — instruccion completa desde qr_node
#   /{veh}/conduccion/block_seguridad BoolStamped   — bloqueos de tof_node y vehicle_node
#
# TOPICOS PUBLICADOS:
#   /{veh}/conduccion/estado          BoolStamped   — True=puede conducir, False=detener
#   /{veh}/conduccion/instruccion     String         — maniobra para lane_follow
#   /{veh}/conduccion/volver_a_casa   BoolStamped   — activa modo retorno
#
# SERVICIOS CONSUMIDOS:
#   /{veh}/led_emitter_node/set_pattern  ChangePattern — control de LEDs
#
# LECCIONES INCORPORADAS DE LOS PROYECTOS ANTERIORES:
#   - No usa rospy.sleep() en callbacks (solo en hilo separado de mision QR)
#   - Un unico productor en conduccion/estado (no como conduccion/block original)
#   - Contador de bloqueos para manejar multiples sensores sin contradiccion
#   - wait_for_service con timeout y manejo de excepcion
#   - Todos los parametros vienen del YAML, no hardcodeados
#   - Maquina de estados explicita con enum (no banderas booleanas sueltas)
# =============================================================================

import sys
sys.path.insert(0, '/home/duckie02/Escritorio/duckieRos2026')

try:
    import rospy
except ImportError:
    import rossimu as rospy

import json
import os
import threading

from enum import Enum, auto

# Stub para módulos de Duckietown
class DTROS:
    def __init__(self, *args, **kwargs):
        pass

class NodeType:
    BEHAVIOR = "BEHAVIOR"
    DRIVER = "DRIVER"
    GENERIC = "GENERIC"

from std_msgs.msg import String
from duckietown_msgs.msg import BoolStamped
from std_msgs.msg import String as DTString

# =============================================================================
# Definicion de estados
# =============================================================================

class Estado(Enum):
    """
    Maquina de estados del sistema.
    Solo el MissionController puede cambiar el estado global.
    """
    DRIVING     = "DRIVING"
    STOPPING    = "STOPPING"
    MANEUVERING = "MANEUVERING"
    BLOCKED     = "BLOCKED"
    QR_ACTION   = "QR_ACTION"
    RETURN_HOME = "RETURN_HOME"
    FINALIZADO  = "FINALIZADO"


# =============================================================================
# Nodo principal
# =============================================================================

class MissionControllerNode(DTROS):
    """
    Orquestador central del sistema Duckiebot para Duckie Day 2026.

    Hereda de DTROS (framework oficial de Duckietown) para integrarse
    correctamente con el ecosistema ROS de la plataforma.
    """

    def __init__(self, node_name):
        super(MissionControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC
        )

        # ─── Nombre del vehiculo ──────────────────────────────────────────
        self.veh = rospy.get_param("~veh", os.environ.get("Myduckiebot02", "duckiebot"))
        rospy.loginfo(f"[MC] Iniciando para vehiculo: {self.veh}")

        # ─── Cargar parametros desde YAML ────────────────────────────────
        self._cargar_parametros()

        # ─── Estado interno ───────────────────────────────────────────────
        self.estado          = Estado.DRIVING
        self.estado_anterior = None

        # Contador de fuentes de bloqueo activas.
        # PROBLEMA que resuelve: si tof Y vehicle_node bloquean simultaneamente,
        # necesitamos que ambos desbloqueen para reanudar (no solo uno).
        self.bloqueos_activos = 0
        self._lock_bloqueo = threading.Lock()

        # Temporizador para la parada ante linea roja
        self._stop_timer = None

        # Temporizador de proteccion para maniobras (evita que el robot
        # quede atascado en MANEUVERING si lane_follow no responde)
        self._maniobra_timer = None

        # Hilo separado para ejecutar las acciones QR (encender/parpadear
        # luces) sin bloquear los callbacks de ROS
        self._qr_thread = None

        # ─── Publicadores ─────────────────────────────────────────────────
        #
        # DISENO: un unico publicador para el estado de conduccion.
        # Esto elimina la condicion de carrera que existia en los proyectos
        # anteriores donde tof_stop, vehicle_detection y detector_qr
        # publicaban todos en el mismo topico /conduccion/block.
        #
        self.pub_estado = rospy.Publisher(
            f"/{self.veh}/conduccion/estado",
            BoolStamped,
            queue_size=1
        )
        self.pub_instruccion = rospy.Publisher(
            f"/{self.veh}/conduccion/instruccion",
            String,
            queue_size=1
        )
        self.pub_volver_casa = rospy.Publisher(
            f"/{self.veh}/conduccion/volver_a_casa",
            BoolStamped,
            queue_size=1
        )
        self.pub_set_instrucciones = rospy.Publisher(
            f"/{self.veh}/conduccion/set_instructions",
            String,
            queue_size=1
        )

        # ─── Suscriptores ─────────────────────────────────────────────────
        #
        # /conduccion/evento: bus de eventos de todos los subsistemas.
        # Cada subsistema publica un JSON con al menos {"tipo": "..."}.
        # El MC decide que hacer con cada evento segun el estado actual.
        #
        rospy.Subscriber(
            f"/{self.veh}/conduccion/evento",
            String,
            self._cb_evento,
            queue_size=10
        )

        # /mision/instrucciones: instruccion completa desde qr_node.
        # Separado de /conduccion/evento porque su procesamiento es mas
        # pesado (puede tomar varios segundos de sleep para los LEDs).
        rospy.Subscriber(
            f"/{self.veh}/mision/instrucciones",
            String,
            self._cb_mision,
            queue_size=1
        )

        # /conduccion/block_seguridad: bloqueos de emergencia de los nodos
        # de seguridad (tof_node y vehicle_node). Usan un topico SEPARADO
        # de /conduccion/evento para garantizar maxima prioridad.
        rospy.Subscriber(
            f"/{self.veh}/conduccion/block_seguridad",
            BoolStamped,
            self._cb_bloqueo_seguridad,
            queue_size=5
        )

        # ─── Servicio LED ─────────────────────────────────────────────────
        # Inicializar servicio con timeout. Si el led_emitter no esta listo,
        # el nodo continua sin LEDs (no se congela como en los proyectos anteriores).
        self.led_svc = None
        self._inicializar_led_service()

        # ─── Publicar estado inicial ──────────────────────────────────────
        rospy.on_shutdown(self._hook_apagado)

        # Dar tiempo a que los suscriptores se conecten antes de publicar
        rospy.sleep(1.0)
        self._publicar_estado(puede_conducir=True)
        self._set_led(self.params_led["DRIVING"])

        rospy.loginfo(f"[MC] ✅ MissionController listo — Estado inicial: {self.estado.value}")

    # =========================================================================
    # Inicializacion de parametros y servicios
    # =========================================================================

    def _cargar_parametros(self):
        """
        Carga todos los parametros desde el YAML de configuracion.
        Usa valores por defecto seguros si el parametro no existe.
        """
        def get(nombre, defecto):
            return rospy.get_param(f"~{nombre}", defecto)

        self.stop_duration          = get("stop_duration", 2.0)
        self.safety_delay           = get("safety_reactivation_delay", 0.5)
        self.maniobra_timeout       = get("maniobra_timeout", 8.0)
        self.qr_action_max_duration = get("qr_action_max_duration", 30.0)
        self.debug_log              = get("debug_log", True)

        self.params_led = get("led_patrones", {
            "DRIVING":     "CAR_DRIVING",
            "STOPPING":    "OBSTACLE_STOPPED",
            "MANEUVERING": "CAR_DRIVING",
            "BLOCKED":     "OBSTACLE_ALERT",
            "QR_ACTION":   "LIGHT_OFF",
            "RETURN_HOME": "BLUE",
            "FINALIZADO":  "LIGHT_OFF",
        })

        self.led_maniobras = get("led_maniobras", {
            "turning_right": "turning_right",
            "turning_left":  "turning_left",
            "move_straight": "move_straight",
            "stop":          "OBSTACLE_STOPPED",
        })

        rospy.loginfo(f"[MC] Parametros cargados — stop_duration={self.stop_duration}s | "
                      f"maniobra_timeout={self.maniobra_timeout}s")

    def _inicializar_led_service(self):
        """
        Intenta conectar al servicio LED con timeout de 10 segundos.
        Si no esta disponible, el nodo continua sin LEDs.
        CORRECCION respecto a proyectos anteriores: no usa wait_for_service
        sin timeout, lo que bloqueaba el nodo indefinidamente.
        """
        srv_name = f"/{self.veh}/led_emitter_node/set_pattern"
        try:
            rospy.wait_for_service(srv_name, timeout=10.0)
            self.led_svc = rospy.ServiceProxy(srv_name, ChangePattern)
            rospy.loginfo("[MC] ✅ Servicio LED conectado correctamente")
        except rospy.ROSException:
            rospy.logwarn("[MC] ⚠️  Servicio LED no disponible — el sistema continua sin LEDs")
            self.led_svc = None

    # =========================================================================
    # Callbacks de suscriptores
    # =========================================================================

    def _cb_evento(self, msg):
        """
        Bus central de eventos del sistema.

        Formato esperado del JSON:
          {"tipo": "linea_roja"}
          {"tipo": "linea_azul"}
          {"tipo": "apriltag", "id": 26, "accion": "turning_right"}
          {"tipo": "maniobra_completa"}

        IMPORTANTE: Los callbacks de ROS se ejecutan en hilos separados por
        suscriptor. Todos los accesos al estado usan chequeos de estado
        para evitar transiciones invalidas.
        """
        try:
            evento = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as e:
            rospy.logwarn(f"[MC] Evento invalido (no es JSON): '{msg.data}' — {e}")
            return

        tipo = evento.get("tipo", "")

        if self.debug_log:
            rospy.loginfo(f"[MC] Evento recibido: {evento} | Estado actual: {self.estado.value}")

        # ── Linea roja detectada por lane_follow ──────────────────────────
        if tipo == "linea_roja":
            if self.estado == Estado.DRIVING:
                self._transicion(Estado.STOPPING)
                self._publicar_estado(puede_conducir=False)
                # Timer para reanudar conduccion automaticamente
                self._cancelar_timer(self._stop_timer)
                self._stop_timer = rospy.Timer(
                    rospy.Duration(self.stop_duration),
                    self._cb_stop_completo,
                    oneshot=True
                )

        # ── Instruccion de AprilTag ───────────────────────────────────────
        elif tipo == "apriltag":
            # Solo reaccionamos si estamos conduciendo normalmente.
            # Si estamos en QR_ACTION o BLOCKED, ignoramos el tag.
            if self.estado == Estado.DRIVING:
                accion = evento.get("accion", "move_straight")
                self._transicion(Estado.MANEUVERING)
                # Publicar instruccion al lane_follow
                self.pub_instruccion.publish(String(data=accion))
                # Activar LED de maniobra
                led_maniobra = self.led_maniobras.get(accion, "CAR_DRIVING")
                self._set_led(led_maniobra)
                # Timer de proteccion: si lane_follow no completa la maniobra
                # en tiempo, volvemos a DRIVING automaticamente
                self._cancelar_timer(self._maniobra_timer)
                self._maniobra_timer = rospy.Timer(
                    rospy.Duration(self.maniobra_timeout),
                    self._cb_maniobra_timeout,
                    oneshot=True
                )

        # ── Maniobra completada (lane_follow avisa cuando termino el giro) ─
        elif tipo == "maniobra_completa":
            if self.estado == Estado.MANEUVERING:
                self._cancelar_timer(self._maniobra_timer)
                self._transicion(Estado.DRIVING)
                self._publicar_estado(puede_conducir=True)

        # ── Linea azul encontrada (modo retorno a casa) ───────────────────
        elif tipo == "linea_azul":
            if self.estado == Estado.RETURN_HOME:
                rospy.logwarn("[MC] 🏠 ¡Linea azul detectada! Robot detenido permanentemente.")
                self._transicion(Estado.FINALIZADO)
                self._publicar_estado(puede_conducir=False)
                # No reactivar nunca mas la conduccion

        else:
            if self.debug_log:
                rospy.loginfo(f"[MC] Tipo de evento no reconocido: '{tipo}'")

    def _cb_mision(self, msg):
        """
        Recibe la instruccion completa desde qr_node.

        Formato JSON esperado (compatible con instrucciones.json local):
          {
            "id": 1,
            "accion": "encender_luces" | "parpadear_luces",
            "tipo_luz": "GREEN",
            "tiempo_detencion": 3.0,
            "repeticiones": 1,
            "retornar_punto_partida": false,
            "nombre_punto": "Estanque A"
          }

        La ejecucion de la mision (sleep para LEDs) se hace en un hilo
        separado para no bloquear el loop de ROS.
        CORRECCION: en los proyectos anteriores, ejecutar_instrucciones.py
        llamaba rospy.sleep() dentro del callback, bloqueando todos los demas
        suscriptores del nodo.
        """
        # Si ya hay una mision ejecutandose, ignorar la nueva
        if self.estado == Estado.QR_ACTION:
            rospy.logwarn("[MC] Ya hay una mision QR en curso — ignorando nueva instruccion")
            return

        # Si el robot esta finalizado, no ejecutar misiones
        if self.estado == Estado.FINALIZADO:
            return

        try:
            instruccion = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as e:
            rospy.logwarn(f"[MC] Instruccion QR invalida: {e}")
            return

        nombre_punto = instruccion.get("nombre_punto", "Desconocido")
        rospy.logwarn(f"[MC] 📦 Mision QR recibida: '{nombre_punto}'")

        # Detener conduccion ANTES de lanzar el hilo
        self._transicion(Estado.QR_ACTION)
        self._publicar_estado(puede_conducir=False)

        # Ejecutar la mision en hilo separado
        self._qr_thread = threading.Thread(
            target=self._ejecutar_mision_qr,
            args=(instruccion,),
            daemon=True
        )
        self._qr_thread.start()

    def _cb_bloqueo_seguridad(self, msg):
        """
        Recibe bloqueos de emergencia de tof_node y vehicle_node.

        DISENO: usa un contador en lugar de un booleano simple.
        Esto permite que multiples nodos de seguridad publiquen bloqueos
        de forma independiente sin contradecirse:
          - Si tof Y vehicle bloquean: bloqueos_activos = 2
          - Cuando tof libera: bloqueos_activos = 1 (sigue bloqueado)
          - Cuando vehicle libera: bloqueos_activos = 0 (se reactiva)

        CORRECCION: en los proyectos anteriores, tof_stop y vehicle_detection
        publicaban en el mismo topico y podian contradecirse.
        """
        with self._lock_bloqueo:
            if msg.data:  # Nuevo bloqueo
                self.bloqueos_activos += 1
                if self.bloqueos_activos == 1:
                    # Primer bloqueo: transicion a BLOCKED
                    self._transicion(Estado.BLOCKED)
                    self._publicar_estado(puede_conducir=False)
                    rospy.logwarn(f"[MC] 🛑 Bloqueo de seguridad activado (total: {self.bloqueos_activos})")
            else:  # Bloqueo liberado
                self.bloqueos_activos = max(0, self.bloqueos_activos - 1)
                rospy.loginfo(f"[MC] Bloqueo liberado (restantes: {self.bloqueos_activos})")
                if self.bloqueos_activos == 0 and self.estado == Estado.BLOCKED:
                    # Todos los sensores liberados: reactivar con delay de seguridad
                    rospy.Timer(
                        rospy.Duration(self.safety_delay),
                        self._cb_reactivar_tras_bloqueo,
                        oneshot=True
                    )

    # =========================================================================
    # Callbacks de timers
    # =========================================================================

    def _cb_stop_completo(self, event):
        """
        Llamado por el timer tras stop_duration segundos de parada ante linea roja.
        Reanuda la conduccion normal.
        """
        if self.estado == Estado.STOPPING:
            rospy.loginfo("[MC] ⏱️  Stop completo — reanudando conduccion")
            self._transicion(Estado.DRIVING)
            self._publicar_estado(puede_conducir=True)

    def _cb_maniobra_timeout(self, event):
        """
        Proteccion: si el lane_follow no informa 'maniobra_completa' en tiempo,
        el MC fuerza el regreso a DRIVING.
        Evita que el robot quede atascado en estado MANEUVERING.
        """
        if self.estado == Estado.MANEUVERING:
            rospy.logwarn("[MC] ⚠️  Timeout de maniobra — forzando regreso a DRIVING")
            self._transicion(Estado.DRIVING)
            self._publicar_estado(puede_conducir=True)

    def _cb_reactivar_tras_bloqueo(self, event):
        """
        Reactiva la conduccion tras un delay de seguridad despues de un bloqueo.
        Solo actua si seguimos en BLOCKED (un nuevo bloqueo podria haber llegado).
        """
        with self._lock_bloqueo:
            if self.bloqueos_activos == 0 and self.estado == Estado.BLOCKED:
                rospy.loginfo("[MC] ✅ Reactivando conduccion tras bloqueo de seguridad")
                self._transicion(Estado.DRIVING)
                self._publicar_estado(puede_conducir=True)

    # =========================================================================
    # Ejecucion de mision QR (en hilo separado)
    # =========================================================================

    def _ejecutar_mision_qr(self, instruccion):
        """
        Ejecuta la accion LED de la mision QR.
        Se ejecuta en un hilo separado para no bloquear los callbacks de ROS.

        CORRECCION respecto a ejecutar_instrucciones.py del proyecto original:
          - No usa rospy.sleep() en el callback principal
          - Tiene timeout maximo de seguridad (qr_action_max_duration)
          - Publica el retorno a casa DESPUES de la accion, no durante
          - La bandera bandera_bloqueado original era redundante y se elimina
        """
        accion       = instruccion.get("accion", "")
        tipo_luz     = instruccion.get("tipo_luz", "white").lower()
        tiempo       = float(instruccion.get("tiempo_detencion", 2.0))
        repeticiones = int(instruccion.get("repeticiones", 1))
        retornar     = instruccion.get("retornar_punto_partida", False)
        nombre       = instruccion.get("nombre_punto", "?")

        # Proteccion: limitar el tiempo maximo de la mision
        tiempo = min(tiempo, self.qr_action_max_duration)

        rospy.logwarn(f"[MC] 💡 Ejecutando mision '{nombre}': {accion} ({tipo_luz})")

        try:
            if accion == "encender_luces":
                self._set_led(tipo_luz)
                rospy.sleep(tiempo)
                self._set_led("LIGHT_OFF")

            elif accion == "parpadear_luces":
                if repeticiones < 1:
                    repeticiones = 1
                t_mitad = (tiempo / repeticiones) / 2.0
                for i in range(repeticiones):
                    self._set_led(tipo_luz)
                    rospy.sleep(t_mitad)
                    self._set_led("LIGHT_OFF")
                    rospy.sleep(t_mitad)
            else:
                rospy.logwarn(f"[MC] Accion QR no reconocida: '{accion}'")
                rospy.sleep(1.0)

        except Exception as e:
            rospy.logerr(f"[MC] Error ejecutando mision QR: {e}")

        # ── Post-mision ───────────────────────────────────────────────────
        if retornar:
            rospy.logwarn(f"[MC] 🏠 Activando modo retorno a casa")
            self._transicion(Estado.RETURN_HOME)
            msg_r = BoolStamped()
            msg_r.header.stamp = rospy.Time.now()
            msg_r.data = True
            self.pub_volver_casa.publish(msg_r)
            # En RETURN_HOME el robot SI puede conducir (busca la linea azul)
            self._publicar_estado(puede_conducir=True)
        else:
            # Mision completada sin retorno: volver a conduccion normal
            rospy.loginfo(f"[MC] ✅ Mision '{nombre}' completada")
            self._transicion(Estado.DRIVING)
            self._publicar_estado(puede_conducir=True)

    # =========================================================================
    # Metodos de utilidad
    # =========================================================================

    def _transicion(self, nuevo_estado):
        """
        Realiza la transicion de estado con logging y cambio de LED.
        Ignora transiciones al mismo estado (no-op).
        """
        if self.estado == nuevo_estado:
            return

        if self.debug_log:
            rospy.logwarn(
                f"[MC] Estado: {self.estado.value} → {nuevo_estado.value}"
            )

        self.estado_anterior = self.estado
        self.estado = nuevo_estado

        # Cambiar LED segun el nuevo estado
        led_pattern = self.params_led.get(nuevo_estado.value, "CAR_DRIVING")
        self._set_led(led_pattern)

    def _publicar_estado(self, puede_conducir):
        """
        Publica el estado de conduccion al lane_follow y demas suscriptores.
        Es el UNICO punto del sistema que publica en este topico.

        puede_conducir=True  → el robot puede moverse
        puede_conducir=False → el robot debe detenerse
        """
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = puede_conducir
        self.pub_estado.publish(msg)

        if self.debug_log:
            estado_str = "✅ PUEDE CONDUCIR" if puede_conducir else "🛑 DETENIDO"
            rospy.loginfo(f"[MC] Publicando estado: {estado_str}")

    def _set_led(self, patron):
        """
        Activa un patron LED en el led_emitter_node.
        Si el servicio no esta disponible, registra un warning y continua.
        """
        if self.led_svc is None:
            return
        try:
            req = ChangePatternRequest()
            req.pattern_name = String(data=patron)
            self.led_svc(req)
        except rospy.ServiceException as e:
            rospy.logwarn(f"[MC] Error al cambiar LED a '{patron}': {e}")

    def _cancelar_timer(self, timer):
        """Cancela un timer de ROS de forma segura si esta activo."""
        if timer is not None:
            try:
                timer.shutdown()
            except Exception:
                pass

    def _hook_apagado(self):
        """
        Callback de apagado: detiene el robot antes de que el nodo termine.
        Publicar varias veces garantiza que el actuador reciba el mensaje.
        """
        rospy.loginfo("[MC] Apagando MissionController — deteniendo robot")
        for _ in range(5):
            self._publicar_estado(puede_conducir=False)
            rospy.sleep(0.05)
        self._set_led("LIGHT_OFF")

# =============================================================================
# Entry point
# =============================================================================

if __name__ == "__main__":
    rospy.init_node("mission_controller_node")
    node = MissionControllerNode(node_name="mission_controller_node")
    rospy.spin()
