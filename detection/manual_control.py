import struct
import time
from ArucoTracker import ArucoTracker, MarkerData
import serial
import math
from typing import Optional, Tuple, Dict
from dataclasses import dataclass

@dataclass
class RobotState:
    """Estado del robot para control centralizado"""
    target_x: Optional[float] = None
    target_y: Optional[float] = None
    current_x: float = 0.0
    current_y: float = 0.0
    current_angle: float = 0.0
    is_moving: bool = False
    last_update: float = 0.0
    last_command: str = ''
    last_command_time: float = 0.0

class RobotController:
    def __init__(self, port='/dev/cu.usbserial-A5069RR4', baudrate=115200):
        try:
            self.serial = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)
            print(f"✓ Conectado a {port}")
        except serial.SerialException as e:
            print(f"✗ Error abriendo puerto serial: {e}")
            raise
        
        self.robot_states: Dict[int, RobotState] = {}
        

    
    def send_command(self, robot_id: int, command: str, speed: int = 600, 
                    x: float = 0.0, y: float = 0.0):
        """Envía comando binario al ESP8266"""
        try:
            packet = struct.pack('<Bchff', 
                                 robot_id, 
                                 command.encode('ascii'), 
                                 int(speed), 
                                 float(x), 
                                 float(y))
            self.serial.write(packet)
            self.serial.flush()
            
            if robot_id in self.robot_states:
                self.robot_states[robot_id].last_command = command
                self.robot_states[robot_id].last_command_time = time.time()
            
            # Solo imprimir comandos de movimiento importantes
            if command in ['L', 'R', 'F', 'B']:
                print(f"→ ID={robot_id} CMD={command} Spd={speed}")
        except Exception as e:
            print(f"❌ Error: {e}")
    


    def stop(self, robot_id: int = 255):
        """Detiene uno o todos los robots"""
        self.send_command(robot_id, 'S', 0)
        if robot_id == 255:
            for state in self.robot_states.values():
                state.is_moving = False
        elif robot_id in self.robot_states:
            self.robot_states[robot_id].is_moving = False
    
    def close(self):
        if self.serial.is_open:
            self.serial.close()
    
    # Añadir contador de debug
    _debug_counter: Dict[int, int] = {}



def test_manual():
    """Modo manual con visualización"""
    
    print("=" * 50)
    print("=== MODO MANUAL ===")
    print("=" * 50)
    
    SERIAL_PORT = '/dev/cu.usbserial-A5069RR4'
    BORDER_IDS = {7, 8, 9, 10}
    WORKSPACE_WIDTH = 1300
    WORKSPACE_HEIGHT = 700
    
    tracker = ArucoTracker(
        marker_length=0.075,
        calib_file="calibracion_charuco.yml",
        border_ids=BORDER_IDS,
        border_order=[10, 9, 8, 7],
        camera_index=0
    )
    
    tracker.start(show_visualization=True)
    time.sleep(2)
    
    try:
        controller = RobotController(port=SERIAL_PORT, baudrate=115200)
    except Exception as e:
        print(f"❌ Error: {e}")
        tracker.stop()
        return
    
    print("\nComandos:")
    print("  f <id> <speed>  - Forward")
    print("  b <id> <speed>  - Backward")
    print("  l <id> <speed>  - Left")
    print("  r <id> <speed>  - Right")
    print("  s <id>          - Stop")
    print("  p               - Posiciones")
    print("  q               - Quit\n")
    
    try:
        while True:
            cmd = input(">>> ").strip().lower().split()
            
            if not cmd:
                continue
            
            if cmd[0] == 'q':
                break
            
            if cmd[0] == 'p':
                markers = tracker.get_all_markers()
                print("\n📍 Posiciones:")
                for robot_id, marker in markers.items():
                    if robot_id not in BORDER_IDS:
                        nx, ny = marker.position
                        nx_c = 1.0 - nx
                        ny_c = 1.0 - ny
                        x = nx_c * WORKSPACE_WIDTH
                        y = ny_c * WORKSPACE_HEIGHT
                        print(f"   Robot {robot_id}: ({x:.0f}, {y:.0f})mm  Ángulo={marker.angle:.0f}°")
                print()
                continue
            
            try:
                if cmd[0] == 'f' and len(cmd) >= 3:
                    controller.send_command(int(cmd[1]), 'F', int(cmd[2]))
                
                elif cmd[0] == 'b' and len(cmd) >= 3:
                    controller.send_command(int(cmd[1]), 'B', int(cmd[2]))
                
                elif cmd[0] == 'l' and len(cmd) >= 3:
                    controller.send_command(int(cmd[1]), 'L', int(cmd[2]))
                
                elif cmd[0] == 'r' and len(cmd) >= 3:
                    controller.send_command(int(cmd[1]), 'R', int(cmd[2]))
                
                elif cmd[0] == 's' and len(cmd) >= 2:
                    controller.send_command(int(cmd[1]), 'S', 0)
                
                else:
                    print("❌ Comando inválido")
            
            except ValueError:
                print("❌ Error en parámetros")
    
    except KeyboardInterrupt:
        print("\n\nSaliendo...")
    
    finally:
        controller.stop(255)
        controller.close()
        tracker.stop()


if __name__ == "__main__":
    print("Modo:")
    print("Modo manual")
    test_manual()
