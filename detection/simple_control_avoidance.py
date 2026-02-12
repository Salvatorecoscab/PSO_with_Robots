
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
    WIDTH_CM = 130.0   
    HEIGHT_CM = 70.0   
    ROBOT_ID = 6
    TARGET_X = 0.1
    TARGET_Y = 0.1
    TARGET_X_CM = TARGET_X * WIDTH_CM  
    TARGET_Y_CM = TARGET_Y * HEIGHT_CM 
    
    ANGLE_TOLERANCE = 18   
    DIST_TOLERANCE_CM = 5.0  
    TURN_PULSE = 0.12    
    MOVE_PULSE = 0.3      # **AUMENTADO** de 0.1 a 0.3 - avanza más antes de recalcular
    SPEED_ANGLE = 400
    SPEED_LINEAR = 600   

    # **DIMENSIONES DEL ROBOT**
    ROBOT_LENGTH_CM = 12.0
    ROBOT_WIDTH_CM = 8.0
    
    # **DISTANCIAS DE SEGURIDAD AJUSTADAS**
    SAFE_DISTANCE_CM = 30.0  # Aumentado para detectar antes
    REPULSION_STRENGTH = 20.0  # Aumentado para reaccionar más fuerte
    EMERGENCY_DISTANCE_CM = 18.0  # Zona crítica más grande
    EMERGENCY_STRENGTH = 80.0  # Repulsión de emergencia muy fuerte
    
    # **FACTOR DE DIRECCIÓN**
    FRONT_MULTIPLIER = 2.0  # Aumentado a 2.0 para esquivar mejor de frente
    
    # **CONTROL DE RECALCULACIÓN**
    RECALC_INTERVAL = 0.15  # Tiempo entre recalculaciones (segundos)
    
    # Factor de escala para normalizar el espacio
    SCALE_FACTOR = WIDTH_CM / HEIGHT_CM
    
    tracker = ArucoTracker(marker_length=0.075, camera_index=0)
    tracker.start(show_visualization=True)
    controller = SimpleRobotController()
    
    last_recalc_time = 0
    last_target_angle = None
    
    try:
        while True:
            markers = tracker.get_all_markers()
            if ROBOT_ID not in markers:
                continue

            marker = markers[ROBOT_ID]
            rx, ry = marker.position[0] * WIDTH_CM, marker.position[1] * HEIGHT_CM
            current_angle = marker.angle % 360
            current_angle_rad = math.radians(current_angle)
            
            current_time = time.time()
            
            # **RECALCULAR SOLO CADA CIERTO INTERVALO**
            if current_time - last_recalc_time >= RECALC_INTERVAL:
                # Normalizar Y para trabajar en espacio cuadrado
                ry_normalized = ry * SCALE_FACTOR
                
                # 1. Vector atracción
                dx = rx - TARGET_X_CM 
                dy = TARGET_Y_CM - ry
                
                # 2. Vector repulsión considerando forma rectangular
                rep_dx = 0
                rep_dy = 0
                for oid, om in markers.items():
                    if oid == ROBOT_ID: continue
                    ox, oy = om.position[0] * WIDTH_CM, om.position[1] * HEIGHT_CM
                    oy_normalized = oy * SCALE_FACTOR
                    
                    # Distancia en espacio normalizado
                    dist = math.hypot(rx - ox, ry_normalized - oy_normalized)
                    
                    # Calcular si el obstáculo está al frente
                    to_obstacle_x = ox - rx
                    to_obstacle_y = oy_normalized - ry_normalized
                    to_obstacle_angle = math.atan2(to_obstacle_y, to_obstacle_x)
                    
                    angle_diff = abs((to_obstacle_angle - current_angle_rad + math.pi) % (2 * math.pi) - math.pi)
                    
                    # Factor direccional
                    direction_factor = 1.0
                    if angle_diff < math.radians(60):
                        direction_factor = FRONT_MULTIPLIER
                    
                    # Zona de emergencia
                    if dist < EMERGENCY_DISTANCE_CM:
                        f = (EMERGENCY_DISTANCE_CM - dist) * EMERGENCY_STRENGTH * direction_factor
                        rep_dx += (ox - rx) / dist * f
                        rep_dy += (ry_normalized - oy_normalized) / dist * f
                    elif dist < SAFE_DISTANCE_CM:
                        f = (SAFE_DISTANCE_CM - dist) * REPULSION_STRENGTH * direction_factor
                        rep_dx += (ox - rx) / dist * f
                        rep_dy += (ry_normalized - oy_normalized) / dist * f
                
                # Desnormalizar rep_dy
                rep_dy = rep_dy / SCALE_FACTOR

                final_dx = dx + rep_dx
                final_dy = dy + rep_dy
                
                last_target_angle = math.degrees(math.atan2(final_dy, final_dx)) % 360
                last_recalc_time = current_time
            
            # Usar el último ángulo calculado
            if last_target_angle is None:
                continue
                
            angle_error = (last_target_angle - current_angle + 180) % 360 - 180
            
            # Verificar si llegó al objetivo
            dx_check = rx - TARGET_X_CM 
            dy_check = TARGET_Y_CM - ry
            dist_real = math.hypot(dx_check, dy_check)

            if dist_real < DIST_TOLERANCE_CM:
                controller.stop(ROBOT_ID)
                break
            
            if abs(angle_error) > ANGLE_TOLERANCE:
                cmd = 'L' if angle_error > 0 else 'R'
                dur = TURN_PULSE if abs(angle_error) < 45 else TURN_PULSE * 2
                controller.pulse_command(ROBOT_ID, cmd, SPEED_ANGLE, dur)
            else:
                controller.pulse_command(ROBOT_ID, 'F', SPEED_LINEAR, MOVE_PULSE)
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        controller.stop(ROBOT_ID)
    finally:
        tracker.stop()
        controller.close()

if __name__ == "__main__":
    control_one_robot()