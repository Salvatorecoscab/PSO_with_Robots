import struct
import time
import serial
import math
from ArucoTracker import ArucoTracker 

class SimpleRobotController:
    def __init__(self, port='/dev/cu.usbserial-A5069RR4', baudrate=115200):
        try:
            self.serial = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(2)
            print(f"✓ Conectado a {port}")
        except Exception as e:
            print(f"❌ Error conectando serial: {e}")
            exit()
    
    def send_command(self, robot_id: int, command: str, speed: int = 600):
        packet = struct.pack('<Bchff', robot_id, command.encode('ascii'), int(speed), 0.0, 0.0)
        self.serial.write(packet)
        self.serial.flush()

    def stop(self, robot_id: int):
        self.send_command(robot_id, 'S', 0)

    def close(self):
        self.serial.close()

def move_robots_to_targets(robot_configs):
    """
    robot_configs: Lista de diccionarios con ID y Target.
    Ej: [{'id': 4, 'target': (0.9, 0.5)}, {'id': 5, 'target': (0.2, 0.2)}]
    """
    # Configuración de parámetros
    ANGLE_TOLERANCE = 12
    DIST_TOLERANCE = 0.03
    TURN_PULSE = 0.1
    MOVE_PULSE = 0.2
    
    tracker = ArucoTracker(marker_length=0.075, camera_index=0)
    tracker.start(show_visualization=True)
    controller = SimpleRobotController()
    
    # Diccionario para rastrear cuándo termina el comando actual de cada robot
    # robot_id -> timestamp de liberación
    busy_until = {cfg['id']: 0 for cfg in robot_configs}
    active_robots = {cfg['id'] for cfg in robot_configs}

    try:
        while active_robots:
            markers = tracker.get_all_markers()
            current_time = time.time()

            for cfg in robot_configs:
                r_id = cfg['id']
                target_x, target_y = cfg['target']

                # 1. Saltamos si el robot ya llegó a su meta
                if r_id not in active_robots:
                    continue

                # 2. No bloqueante: si el robot aún está ejecutando un pulso, pasamos al siguiente
                if current_time < busy_until[r_id]:
                    continue

                # 3. Verificar si el tracker ve al robot
                if r_id not in markers:
                    continue

                marker = markers[r_id]
                nx, ny = marker.position 
                current_angle = marker.angle % 360

                # Cálculo de trayectoria
                dx = nx - target_x 
                dy = target_y - ny
                target_angle_deg = math.degrees(math.atan2(dy, dx)) % 360
                angle_error = (target_angle_deg - current_angle + 180) % 360 - 180
                distance = math.hypot(dx, dy)

                # Lógica de estados
                if distance < DIST_TOLERANCE:
                    print(f"🎯 Robot {r_id} llegó a su destino.")
                    controller.stop(r_id)
                    active_robots.remove(r_id)
                    continue
                
                if abs(angle_error) > ANGLE_TOLERANCE:
                    cmd = 'L' if angle_error > 0 else 'R'
                    duration = TURN_PULSE if abs(angle_error) < 45 else TURN_PULSE * 1.5
                    controller.send_command(r_id, cmd, 400)
                    busy_until[r_id] = current_time + duration
                else:
                    controller.send_command(r_id, 'F', 600)
                    busy_until[r_id] = current_time + MOVE_PULSE
            
            # Pequeño respiro para el procesador
            time.sleep(0.01)

    except KeyboardInterrupt:
        for cfg in robot_configs: controller.stop(cfg['id'])
    finally:
        tracker.stop()
        controller.close()

if __name__ == "__main__":
    # Define aquí tus 3 robots y sus destinos
    mis_robots = [
        {'id': 4, 'target': (0.9, 0.5)},
        {'id': 5, 'target': (0.1, 0.1)},
        {'id': 6, 'target': (0.5, 0.8)}
    ]
    
    move_robots_to_targets(mis_robots)