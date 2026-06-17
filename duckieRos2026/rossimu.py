#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS Simulator - Simulador simplificado de ROS para ejecución local
==================================================================

Proporciona una interfaz simulada de ROS para ejecutar nodos sin
necesidad de tener ROS instalado en el sistema.
"""

import json
import threading
import queue
import time
from typing import Dict, Callable, Any, List
from pathlib import Path
from datetime import datetime

class Message:
    """Clase base para mensajes ROS."""
    def __init__(self, data=None):
        self.data = data
        self.header = {"stamp": time.time()}

class Publisher:
    """Publicador de tópicos."""
    def __init__(self, topic_name: str, message_queue: Dict[str, queue.Queue]):
        self.topic_name = topic_name
        self.message_queue = message_queue
    
    def publish(self, msg):
        """Publica un mensaje al tópico."""
        if self.topic_name not in self.message_queue:
            self.message_queue[self.topic_name] = queue.Queue(maxsize=10)
        
        try:
            self.message_queue[self.topic_name].put_nowait(msg)
        except queue.Full:
            self.message_queue[self.topic_name].get_nowait()
            self.message_queue[self.topic_name].put_nowait(msg)

class Subscriber:
    """Suscriptor de tópicos."""
    def __init__(self, topic_name: str, message_queue: Dict[str, queue.Queue], callback: Callable):
        self.topic_name = topic_name
        self.message_queue = message_queue
        self.callback = callback
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.running = True
        self.thread.start()
    
    def _listen(self):
        """Escucha mensajes del tópico."""
        if self.topic_name not in self.message_queue:
            self.message_queue[self.topic_name] = queue.Queue()
        
        while self.running:
            try:
                msg = self.message_queue[self.topic_name].get(timeout=0.1)
                self.callback(msg)
            except queue.Empty:
                pass
    
    def shutdown(self):
        """Detiene el suscriptor."""
        self.running = False

class ROSSimulator:
    """Simulador ROS central."""
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
        
        self.message_queues: Dict[str, queue.Queue] = {}
        self.nodes: Dict[str, Any] = {}
        self.running = True
        self._initialized = True
        
        print("[ROSSimulator] Sistema iniciado")
    
    def init_node(self, node_name: str):
        """Inicializa un nodo."""
        self.nodes[node_name] = {
            "name": node_name,
            "created_at": time.time(),
            "publishers": {},
            "subscribers": {},
            "params": {}
        }
        print(f"[{node_name}] Nodo iniciado")
    
    def Publisher(self, topic: str, queue_size: int = 1):
        """Crea un publicador."""
        return Publisher(topic, self.message_queues)
    
    def Subscriber(self, topic: str, callback: Callable):
        """Crea un suscriptor."""
        return Subscriber(topic, self.message_queues, callback)
    
    def get_param(self, param_name: str, default=None):
        """Obtiene un parámetro ROS."""
        params = {
            "~kp": 1.5,
            "~ki": 0.1,
            "~kd": 0.5,
            "~max_linear_velocity": 0.3,
            "~max_angular_velocity": 1.5,
            "~min_linear_velocity": 0.05,
            "~veh": "duckiebot",
            "~processing_rate": 30,
            "~debug_mode": False,
        }
        return params.get(param_name, default)
    
    def Rate(self, hz: int):
        """Crea un rate limitador."""
        return RateObject(hz)
    
    def is_shutdown(self):
        """Verifica si el sistema está en shutdown."""
        return not self.running
    
    def shutdown(self):
        """Apaga el simulador."""
        self.running = False
        print("[ROSSimulator] Sistema apagado")

class RateObject:
    """Objeto para limitar la frecuencia de ejecución."""
    def __init__(self, hz: int):
        self.interval = 1.0 / hz
        self.last_time = time.time()
    
    def sleep(self):
        """Duerme hasta completar el intervalo."""
        elapsed = time.time() - self.last_time
        if elapsed < self.interval:
            time.sleep(self.interval - elapsed)
        self.last_time = time.time()

class Time:
    """Clase para manejar tiempo en ROS."""
    @staticmethod
    def now():
        return time.time()

# API global compatible con rospy
_ros_sim = ROSSimulator()

def init_node(name, anonymous=False):
    """Inicializa un nodo ROS."""
    _ros_sim.init_node(name)

def loginfo(msg):
    """Registra un mensaje de información."""
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] [INFO] {msg}")

def logwarn(msg):
    """Registra una advertencia."""
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] [WARN] {msg}")

def logerr(msg):
    """Registra un error."""
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] [ERR] {msg}")

def get_param(name, default=None):
    """Obtiene un parámetro ROS."""
    return _ros_sim.get_param(name, default)

def Publisher(topic, message_type=None, queue_size=1):
    """Crea un publicador."""
    return _ros_sim.Publisher(topic, queue_size)

def Subscriber(topic, message_type, callback, queue_size=1):
    """Crea un suscriptor."""
    return _ros_sim.Subscriber(topic, callback)

def Rate(hz):
    """Crea un limitador de frecuencia."""
    return _ros_sim.Rate(hz)

def is_shutdown():
    """Verifica si hay shutdown."""
    return _ros_sim.is_shutdown()

class ROSInterruptException(Exception):
    """Excepción de interrupción de ROS."""
    pass
