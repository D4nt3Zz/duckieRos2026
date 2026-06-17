#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Lane Follow Node - Duckiebot DB21J
==================================

Nodo de seguimiento de carril para Duckiebot DB21J.

El nodo procesa imágenes de la cámara frontal para detectar carriles
(línea blanca a la izquierda, línea amarilla a la derecha) y calcula
comandos de velocidad angular para mantener el robot centrado en el carril.

INTERFACES ROS:
    Suscriptores:
    - /veh/camera_node/image/compressed (sensor_msgs/CompressedImage)
    - /veh/conduccion/instruccion (String) — instrucciones del mission_controller
    
    Publicadores:
    - /veh/wheels_driver_node/wheels_cmd (duckietown_msgs/WheelsCmd)
    - /veh/conduccion/maniobra_completada (String) — notificar maniobra completada
    - /veh/conduccion/evento (String) — eventos de detección
    - /veh/lane_follow_node/debug_image (sensor_msgs/Image) — [debug]

PARÁMETROS:
    veh: Nombre del robot (Duckiebot)
    ~kp, ~ki, ~kd: Ganancias PID
    ~max_linear_velocity: Velocidad lineal máxima
    ~max_angular_velocity: Velocidad angular máxima

COMPORTAMIENTO:
    1. Detecta líneas blancas (izquierda) y amarillas (derecha)
    2. Calcula desviación del centro del carril
    3. Usa control PID para corregir la trayectoria
    4. Publica comandos a los motores
"""

import sys
sys.path.insert(0, '/home/duckie02/Escritorio/duckieRos2026')

try:
    import rospy
except ImportError:
    import rossimu as rospy

import cv2
import numpy as np
try:
    from cv_bridge import CvBridge, CvBridgeError
except ImportError:
    class CvBridge:
        def imdecode(self, data): return None
        def cv2_to_imgmsg(self, cv_image, encoding): return None
    class CvBridgeError(Exception): pass

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from duckietown_msgs.msg import WheelsCmd
import json
from threading import Lock
from time import time

class LaneFollowNode:
    """Nodo principal de seguimiento de carril."""
    
    def __init__(self):
        """Inicializa el nodo de lane following."""
        rospy.init_node('lane_follow_node', anonymous=False)
        
        # Obtener parámetros
        self.veh = rospy.get_param('~veh', 'duckiebot')
        
        # Parámetros de control PID
        self.kp = rospy.get_param('~kp', 1.5)
        self.ki = rospy.get_param('~ki', 0.1)
        self.kd = rospy.get_param('~kd', 0.5)
        self.integral_max = rospy.get_param('~integral_max', 1.0)
        
        # Parámetros de velocidad
        self.max_linear_v = rospy.get_param('~max_linear_velocity', 0.3)
        self.max_angular_v = rospy.get_param('~max_angular_velocity', 1.5)
        self.min_linear_v = rospy.get_param('~min_linear_velocity', 0.05)
        
        # Parámetros de detección
        self.processing_rate = rospy.get_param('~processing_rate', 30)
        self.debug_mode = rospy.get_param('~debug_mode', False)
        
        # Variables de estado
        self.last_error = 0.0
        self.integral_error = 0.0
        self.last_time = time()
        self.current_instruction = "lane_follow"  # Por defecto, seguir carril
        self.is_maneuvering = False
        
        # Locks para acceso seguro a variables compartidas
        self.image_lock = Lock()
        self.state_lock = Lock()
        self.current_image = None
        
        # Bridge de OpenCV
        self.bridge = CvBridge()
        
        # Suscriptores
        rospy.Subscriber(
            f'/{self.veh}/camera_node/image/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=1
        )
        
        rospy.Subscriber(
            f'/{self.veh}/conduccion/instruccion',
            String,
            self.instruction_callback
        )
        
        # Publicadores
        self.wheels_pub = rospy.Publisher(
            f'/{self.veh}/wheels_driver_node/wheels_cmd',
            WheelsCmd,
            queue_size=1
        )
        
        self.maniobra_pub = rospy.Publisher(
            f'/{self.veh}/conduccion/maniobra_completada',
            String,
            queue_size=1
        )
        
        self.evento_pub = rospy.Publisher(
            f'/{self.veh}/conduccion/evento',
            String,
            queue_size=1
        )
        
        if self.debug_mode:
            self.debug_pub = rospy.Publisher(
                f'/{self.veh}/lane_follow_node/debug_image',
                Image,
                queue_size=1
            )
        
        rospy.loginfo(f"[{self.veh}] Lane Follow Node iniciado")
        
    def image_callback(self, msg):
        """Callback para procesar imágenes comprimidas."""
        try:
            # Descomprimir imagen
            cv_image = cv2.imdecode(
                np.frombuffer(msg.data, np.uint8),
                cv2.IMREAD_COLOR
            )
            
            with self.image_lock:
                self.current_image = cv_image
                
        except Exception as e:
            rospy.logwarn(f"Error descomprimiendo imagen: {e}")
    
    def instruction_callback(self, msg):
        """Recibe instrucciones del mission_controller."""
        with self.state_lock:
            self.current_instruction = msg.data
            
        if "turn_" in msg.data or "maniobra_" in msg.data:
            with self.state_lock:
                self.is_maneuvering = True
            rospy.loginfo(f"[{self.veh}] Maniobra recibida: {msg.data}")
    
    def detect_lanes(self, image):
        """
        Detecta líneas blancas (izquierda) y amarillas (derecha).
        
        Retorna:
            tuple: (imagen_procesada, desviacion_del_centro, confianza)
        """
        if image is None:
            return None, 0.0, 0.0
        
        # Convertir a HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        height, width = image.shape[:2]
        
        # ROI: área inferior de la imagen (donde se ve el carril)
        roi_top = int(height * 0.5)
        roi = hsv[roi_top:, :]
        
        # Detectar línea blanca (izquierda)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        white_mask = cv2.inRange(roi, lower_white, upper_white)
        
        # Detectar línea amarilla (derecha)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(roi, lower_yellow, upper_yellow)
        
        # Combinar máscaras
        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # Aplicar morfología
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
        
        # Encontrar contornos
        contours, _ = cv2.findContours(
            lane_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Calcular centro del carril
        center_deviation = 0.0
        confidence = 0.0
        
        if contours:
            # Obtener el contorno más grande
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > 100:  # Área mínima
                # Calcular momento
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    # Desviación respecto al centro de la imagen
                    image_center = width // 2
                    center_deviation = (cx - image_center) / image_center
                    confidence = min(1.0, area / (width * len(roi)))
        
        # Debug: generar imagen procesada
        if self.debug_mode:
            debug_image = cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR)
            return debug_image, center_deviation, confidence
        
        return None, center_deviation, confidence
    
    def calculate_pid(self, error, dt):
        """Calcula la corrección PID."""
        # Proporcional
        p_term = self.kp * error
        
        # Integral
        self.integral_error += error * dt
        self.integral_error = np.clip(
            self.integral_error,
            -self.integral_max,
            self.integral_max
        )
        i_term = self.ki * self.integral_error
        
        # Derivativa
        d_term = self.kd * (error - self.last_error) / dt if dt > 0 else 0.0
        
        self.last_error = error
        
        # Comando angular total
        angular_cmd = p_term + i_term + d_term
        angular_cmd = np.clip(angular_cmd, -self.max_angular_v, self.max_angular_v)
        
        return angular_cmd
    
    def run(self):
        """Loop principal del nodo."""
        rate = rospy.Rate(self.processing_rate)
        
        while not rospy.is_shutdown():
            try:
                # Obtener imagen actual
                with self.image_lock:
                    image = self.current_image
                    self.current_image = None
                
                if image is None:
                    rate.sleep()
                    continue
                
                # Detectar carriles
                debug_img, deviation, confidence = self.detect_lanes(image)
                
                # Obtener instrucción actual
                with self.state_lock:
                    instruction = self.current_instruction
                
                # Calcular comandos basado en instrucción
                if instruction == "lane_follow" or instruction == "driving":
                    # Calcular tiempo desde última iteración
                    now = time()
                    dt = now - self.last_time
                    self.last_time = now
                    
                    # PID control
                    angular_cmd = self.calculate_pid(deviation, dt)
                    linear_cmd = self.max_linear_v
                    
                    # Reducir velocidad si confianza es baja
                    if confidence < 0.3:
                        linear_cmd *= 0.5
                        rospy.logwarn(f"[{self.veh}] Baja confianza en detección")
                    
                else:
                    # Detener durante maniobra
                    linear_cmd = 0.0
                    angular_cmd = 0.0
                
                # Publicar comandos a los motores
                wheels_msg = WheelsCmd()
                wheels_msg.header.stamp = rospy.Time.now()
                
                # Convertir a comandos de velocidad de ruedas
                # Para diferencial drive: v_left = v_linear - omega * width/2
                #                         v_right = v_linear + omega * width/2
                wheel_distance = 0.1  # Ancho de la base del robot (metros)
                
                left_speed = linear_cmd - (angular_cmd * wheel_distance / 2)
                right_speed = linear_cmd + (angular_cmd * wheel_distance / 2)
                
                wheels_msg.vel_left = np.clip(left_speed, -1.0, 1.0)
                wheels_msg.vel_right = np.clip(right_speed, -1.0, 1.0)
                
                self.wheels_pub.publish(wheels_msg)
                
                # Publicar evento si confianza es baja
                if confidence < 0.2 and confidence > 0.0:
                    evento = {
                        "tipo": "lane_detection_low_confidence",
                        "confianza": float(confidence),
                        "desviacion": float(deviation)
                    }
                    self.evento_pub.publish(String(json.dumps(evento)))
                
                # Publicar imagen de debug si está activado
                if self.debug_mode and debug_img is not None:
                    try:
                        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
                        self.debug_pub.publish(debug_msg)
                    except CvBridgeError as e:
                        rospy.logwarn(f"Error publicando imagen de debug: {e}")
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error en loop principal: {e}")
                rate.sleep()

if __name__ == '__main__':
    try:
        node = LaneFollowNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
