#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Local Launcher - Ejecuta nodos ROS localmente sin ROS instalado
================================================================

Permite lanzar múltiples nodos Python ROS de forma simulada
sin requerir una instalación completa de ROS.

Uso:
    python3 launcher.py lane_follow veh=myduckiebot02
    python3 launcher.py mission_controller veh=myduckiebot02
    python3 launcher.py all veh=myduckiebot02
"""

import sys
import os
import subprocess
import signal
import argparse
import time
from pathlib import Path
from typing import Dict, List

class LocalLauncher:
    """Gestor de lanzamiento local de nodos."""
    
    WORKSPACE_ROOT = Path("/home/duckie02/Escritorio/duckieRos2026")
    
    NODES = {
        "mission_controller": {
            "package": "mission_controller",
            "script": "mission_controller_node.py",
            "required": True,
        },
        "lane_follow": {
            "package": "lane_follow",
            "script": "lane_follow_node.py",
            "required": True,
        },
        "led_emitter": {
            "package": "led_emitter",
            "script": "led_emitter_node.py",
            "required": False,
        },
    }
    
    def __init__(self, veh: str = "duckiebot"):
        """Inicializa el lanzador."""
        self.veh = veh
        self.processes: Dict[str, subprocess.Popen] = {}
        self.running = True
        
        # Configurar señales
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        print(f"[Launcher] Inicializando con veh={veh}")
    
    def _signal_handler(self, signum, frame):
        """Maneja señales de interrupción."""
        print("\n[Launcher] Recibida señal de interrupción")
        self.shutdown()
        sys.exit(0)
    
    def launch_node(self, node_name: str) -> bool:
        """Lanza un nodo específico."""
        if node_name not in self.NODES:
            print(f"[Launcher] ERROR: Nodo desconocido: {node_name}")
            return False
        
        node_config = self.NODES[node_name]
        package = node_config["package"]
        script = node_config["script"]
        
        script_path = self.WORKSPACE_ROOT / "packages" / package / "src" / script
        
        if not script_path.exists():
            if node_config["required"]:
                print(f"[Launcher] ERROR: No se encontró {script_path}")
                return False
            else:
                print(f"[Launcher] SKIP: {node_name} (no encontrado, no obligatorio)")
                return True
        
        print(f"[Launcher] Lanzando {node_name}...")
        
        try:
            # Crear proceso para el nodo
            env = os.environ.copy()
            env["PYTHONPATH"] = str(self.WORKSPACE_ROOT) + ":" + env.get("PYTHONPATH", "")
            env["VEHICLE_NAME"] = self.veh
            env["ROS_NAMESPACE"] = self.veh
            
            # Usar Python del venv si existe
            venv_python = self.WORKSPACE_ROOT / ".venv" / "bin" / "python"
            python_exec = str(venv_python) if venv_python.exists() else sys.executable
            
            proc = subprocess.Popen(
                [python_exec, str(script_path)],
                cwd=str(self.WORKSPACE_ROOT),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
            )
            
            self.processes[node_name] = proc
            
            # Thread para monitorear la salida
            import threading
            def log_output():
                try:
                    for line in proc.stdout:
                        print(f"[{node_name}] {line.rstrip()}")
                except:
                    pass
            
            thread = threading.Thread(target=log_output, daemon=True)
            thread.start()
            
            print(f"[Launcher] {node_name} iniciado (PID: {proc.pid})")
            return True
            
        except Exception as e:
            print(f"[Launcher] ERROR al lanzar {node_name}: {e}")
            return False
    
    def launch_nodes(self, node_names: List[str]) -> bool:
        """Lanza múltiples nodos."""
        success = True
        for node_name in node_names:
            if not self.launch_node(node_name):
                if self.NODES[node_name]["required"]:
                    success = False
        
        return success
    
    def shutdown(self):
        """Apaga todos los nodos."""
        print("\n[Launcher] Apagando nodos...")
        
        for node_name, proc in list(self.processes.items()):
            try:
                print(f"[Launcher] Terminando {node_name}...")
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                print(f"[Launcher] {node_name} terminado")
            except Exception as e:
                print(f"[Launcher] Error terminando {node_name}: {e}")
        
        self.processes.clear()
        print("[Launcher] Todos los nodos apagados")
    
    def wait(self):
        """Espera a que los procesos terminen."""
        try:
            while self.running:
                # Verificar si algún proceso terminó
                for node_name, proc in list(self.processes.items()):
                    if proc.poll() is not None:
                        retcode = proc.returncode
                        print(f"[Launcher] {node_name} terminó con código {retcode}")
                        if self.NODES[node_name]["required"] and retcode != 0:
                            print("[Launcher] Nodo requerido falló, apagando sistema")
                            self.shutdown()
                            return False
                
                time.sleep(1)
        except KeyboardInterrupt:
            self.shutdown()
            return False
        
        return True

def main():
    """Función principal."""
    parser = argparse.ArgumentParser(
        description="Local ROS Node Launcher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos:
  python3 launcher.py lane_follow veh=myduckiebot02
  python3 launcher.py mission_controller veh=myduckiebot02
  python3 launcher.py all veh=myduckiebot02
        """
    )
    
    parser.add_argument(
        "node",
        nargs="?",
        default="all",
        help="Nodo o 'all' para lanzar todos (default: all)"
    )
    parser.add_argument(
        "args",
        nargs="*",
        help="Argumentos adicionales (ej: veh=myduckiebot02)"
    )
    
    args = parser.parse_args()
    
    # Parsear argumentos
    veh = "duckiebot"
    for arg in args.args:
        if "=" in arg:
            key, value = arg.split("=", 1)
            if key == "veh":
                veh = value
    
    # Crear launcher
    launcher = LocalLauncher(veh=veh)
    
    # Determinar qué nodos lanzar
    if args.node == "all":
        nodes_to_launch = list(launcher.NODES.keys())
    else:
        nodes_to_launch = [args.node]
    
    # Lanzar nodos
    if not launcher.launch_nodes(nodes_to_launch):
        print("[Launcher] Fallo en el lanzamiento de nodos requeridos")
        return 1
    
    print(f"\n[Launcher] Sistema lanzado exitosamente")
    print(f"[Launcher] Presiona Ctrl+C para apagar\n")
    
    # Esperar
    if not launcher.wait():
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
