#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LED Emitter Node - Duckiebot DB21J
===================================

Nodo controlador de LEDs para Duckiebot DB21J.

El nodo gestiona los patrones de iluminación frontal del robot
basado en instrucciones recibidas desde el mission_controller.

INTERFACES ROS:
    Suscriptores:
    - /veh/led_emitter_node/set_pattern (duckietown_msgs/ChangePattern)
    - /veh/conduccion/estado (String) — estado del sistema
    
    Publicadores:
    - /veh/led_emitter_node/status (String) — estado actual del controlador

PARÁMETROS:
    veh: Nombre del robot
    ~num_leds: Número de LEDs
    ~pwm_frequency: Frecuencia PWM

PATRONES DISPONIBLES:
    - solid_white: LED blanco sólido
    - solid_red: LED rojo sólido
    - blink_white: LED blanco parpadeante
    - blink_yellow: LED amarillo parpadeante
    - blink_red: LED rojo parpadeante
"""

import sys
sys.path.insert(0, '/home/duckie02/Escritorio/duckieRos2026')

try:
    import rospy
except ImportError:
    import rossimu as rospy

from std_msgs.msg import String
import json
from time import time, sleep
import threading

class LEDEmitterNode:
    """Nodo controlador de LEDs."""
    
    # Definición de patrones
    PATTERNS = {
        "solid_white": {"color": [255, 255, 255], "blink": False, "frequency": 0},
        "solid_red": {"color": [255, 0, 0], "blink": False, "frequency": 0},
        "solid_green": {"color": [0, 255, 0], "blink": False, "frequency": 0},
        "blink_white": {"color": [255, 255, 255], "blink": True, "frequency": 2},
        "blink_yellow": {"color": [255, 255, 0], "blink": True, "frequency": 1},
        "blink_red": {"color": [255, 0, 0], "blink": True, "frequency": 2},
        "off": {"color": [0, 0, 0], "blink": False, "frequency": 0},
    }
    
    def __init__(self):
        """Inicializa el nodo controlador de LEDs."""
        rospy.init_node('led_emitter_node', anonymous=False)
        
        # Parámetros
        self.veh = rospy.get_param('~veh', 'duckiebot')
        self.num_leds = rospy.get_param('~num_leds', 2)
        self.pwm_freq = rospy.get_param('~pwm_frequency', 1000)
        
        # Estado
        self.current_pattern = "off"
        self.pattern_active = False
        self.last_pattern_time = time()
        
        # Publicadores
        self.status_pub = rospy.Publisher(
            f'/{self.veh}/led_emitter_node/status',
            String,
            queue_size=1
        )
        
        # Suscriptores
        rospy.Subscriber(
            f'/{self.veh}/led_emitter_node/set_pattern',
            String,
            self.pattern_callback
        )
        
        rospy.Subscriber(
            f'/{self.veh}/conduccion/estado',
            String,
            self.estado_callback
        )
        
        rospy.loginfo(f"[{self.veh}] LED Emitter Node iniciado")
        rospy.loginfo(f"[{self.veh}] LEDs configurados: {self.num_leds}")
    
    def pattern_callback(self, msg):
        """Callback para cambiar patrón de LEDs."""
        try:
            data = json.loads(msg.data)
            pattern_name = data.get("pattern", "off")
            
            if pattern_name in self.PATTERNS:
                self.current_pattern = pattern_name
                rospy.loginfo(f"[{self.veh}] Patrón LED: {pattern_name}")
                
                # Publicar estado
                status = {
                    "pattern": pattern_name,
                    "active": True,
                    "timestamp": time()
                }
                self.status_pub.publish(String(json.dumps(status)))
            else:
                rospy.logwarn(f"[{self.veh}] Patrón desconocido: {pattern_name}")
        
        except json.JSONDecodeError:
            rospy.logwarn(f"[{self.veh}] Formato JSON inválido en patrón")
        except Exception as e:
            rospy.logerr(f"[{self.veh}] Error en pattern_callback: {e}")
    
    def estado_callback(self, msg):
        """Callback para estado del sistema."""
        try:
            estado = msg.data
            
            # Cambiar LED según estado
            if estado == "STOPPED":
                self.current_pattern = "solid_red"
            elif estado == "DRIVING":
                self.current_pattern = "solid_white"
            elif estado == "MANEUVERING":
                self.current_pattern = "blink_yellow"
            elif estado == "ERROR":
                self.current_pattern = "blink_red"
            
            rospy.loginfo(f"[{self.veh}] Estado: {estado} -> Patrón: {self.current_pattern}")
        
        except Exception as e:
            rospy.logerr(f"[{self.veh}] Error en estado_callback: {e}")
    
    def render_led(self):
        """Renderiza el patrón actual en los LEDs."""
        pattern = self.PATTERNS.get(self.current_pattern, self.PATTERNS["off"])
        
        if pattern["blink"]:
            # Efecto parpadeo
            elapsed = (time() - self.last_pattern_time) % (1.0 / pattern["frequency"])
            if elapsed < (1.0 / pattern["frequency"]) / 2:
                # LED encendido
                return pattern["color"]
            else:
                # LED apagado
                return [0, 0, 0]
        else:
            # LED sólido
            return pattern["color"]
    
    def run(self):
        """Loop principal del nodo."""
        rate = rospy.Rate(30)  # 30 Hz
        
        while not rospy.is_shutdown():
            try:
                # Renderizar LED
                color = self.render_led()
                
                # Aquí iría el código real para controlar el hardware
                # Por ahora solo simulamos
                
                rate.sleep()
            
            except Exception as e:
                rospy.logerr(f"[{self.veh}] Error en loop: {e}")
                rate.sleep()

if __name__ == '__main__':
    try:
        node = LEDEmitterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
