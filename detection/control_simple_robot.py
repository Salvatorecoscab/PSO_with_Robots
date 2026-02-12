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
    
    def pulse_command(self, robot_id: int, command: str, speed: int, duration: float):
        self.send_command(robot_id, command, speed)
        time.sleep(duration)
        self.stop(robot_id)

    def close(self):
        self.serial.close()

def control_one_robot():
    # --- CONFIGURACIÓN DE ESPACIO REAL ---
    WIDTH_CM = 130.0   # Ancho de la mesa
    HEIGHT_CM = 70.0   # Alto de la mesa
    
    ROBOT_ID = 6
    TARGET_X=0.1
    TARGET_Y=0.1
    # Definimos el objetivo directamente en centímetros para que sea más intuitivo
    TARGET_X_CM = TARGET_X * WIDTH_CM  # Convertir a cm
    TARGET_Y_CM = TARGET_Y * HEIGHT_CM # Convertir a cm
    
    # Tolerancias en centímetros
    ANGLE_TOLERANCE = 18   
    DIST_TOLERANCE_CM = 5.0  # Detenerse a 5cm del objetivo
    
    TURN_PULSE = 0.12    
    MOVE_PULSE = 0.1      
    SPEED_ANGLE = 400
    SPEED_LINEAR = 600   
    
    tracker = ArucoTracker(marker_length=0.075, camera_index=0)
    tracker.start(show_visualization=True)
    controller = SimpleRobotController()
    
    try:
        while True:
            markers = tracker.get_all_markers()
            if ROBOT_ID not in markers:
                continue

            marker = markers[ROBOT_ID]
            nx, ny = marker.position 
            current_angle = marker.angle % 360

            # --- CONVERSIÓN A COORDENADAS REALES ---
            rx = nx * WIDTH_CM
            ry = ny * HEIGHT_CM

            # Cálculo de distancia y ángulo usando valores en CM
            dx = rx - TARGET_X_CM 
            dy = TARGET_Y_CM - ry
            
            target_angle_rad = math.atan2(dy, dx)
            target_angle_deg = math.degrees(target_angle_rad) % 360
            
            angle_error = (target_angle_deg - current_angle + 180) % 360 - 180
            distance_cm = math.hypot(dx, dy)

            # Print con información real
            print(f"Pos Real: ({rx:.1f}, {ry:.1f})cm | Dist: {distance_cm:.1f}cm | Error Ang: {angle_error:.1f}°")

            # --- LÓGICA DE MOVIMIENTO ---
            if distance_cm < DIST_TOLERANCE_CM:
                print(f"🎯 ¡Llegamos al objetivo físico ({TARGET_X_CM}, {TARGET_Y_CM})!")
                controller.stop(ROBOT_ID)
                break
            
            if abs(angle_error) > ANGLE_TOLERANCE:
                cmd = 'L' if angle_error > 0 else 'R'
                duration = TURN_PULSE if abs(angle_error) < 45 else TURN_PULSE * 2
                controller.pulse_command(ROBOT_ID, cmd, SPEED_ANGLE, duration)
            else:
                controller.pulse_command(ROBOT_ID, 'F', SPEED_LINEAR, MOVE_PULSE)
            
            time.sleep(0.1)

    except KeyboardInterrupt:
        controller.stop(ROBOT_ID)
    finally:
        tracker.stop()
        controller.close()

if __name__ == "__main__":
    control_one_robot()