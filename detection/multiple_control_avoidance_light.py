import struct
import time
import serial
import math
from ArucoTracker import ArucoTracker 
import threading
import random

class SimpleRobotController:
    def __init__(self, port='/dev/cu.usbserial-A5069RR4', baudrate=115200):
        try:
            self.serial = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(2)
            print(f"✓ Conectado a {port}")
            self.light_readings = {}
            self.light_lock = threading.Lock() 
            self.sensor_stats = {'received': 0, 'errors': 0}

            self.running = True
            self.sensor_thread = threading.Thread(target=self._read_sensors, daemon=True)
            self.sensor_thread.start()
            
        except Exception as e:
            print(f"❌ Error conectando serial: {e}")
            exit()
    def _read_sensors(self):
        """Thread que lee continuamente los datos de sensores de forma segura"""
        SENSOR_MSG_SIZE = 4
        buffer = bytearray()
        VALID_ROBOT_IDS = {4, 5, 6}
        
        while self.running:
            try:
                if self.serial.in_waiting > 0:
                    chunk = self.serial.read(self.serial.in_waiting)
                    buffer.extend(chunk)
                while len(buffer) >= SENSOR_MSG_SIZE:
                    try:
                        # Formato: <B (robot_id) h (light_value) x (padding)
                        robot_id, light_value = struct.unpack('<Bh', buffer[:3])
                        
                        # Validar que el ID sea válido (evita basura)
                        if robot_id in VALID_ROBOT_IDS:
                            # Validar rango de luz (0-1023 para ADC de 10 bits)
                            if 0 <= light_value <= 1023:
                                with self.light_lock:
                                    self.light_readings[robot_id] = light_value
                                
                                self.sensor_stats['received'] += 1
                                
                                # Remover mensaje procesado
                                buffer = buffer[SENSOR_MSG_SIZE:]
                            else:
                                # Valor fuera de rango, descartar primer byte y reintentar
                                buffer = buffer[1:]
                                self.sensor_stats['errors'] += 1
                        else:
                            # ID inválido, descartar primer byte
                            buffer = buffer[1:]
                            self.sensor_stats['errors'] += 1
                    
                    except struct.error:
                        # Error de desempaquetado, descartar primer byte
                        buffer = buffer[1:]
                        self.sensor_stats['errors'] += 1
                
                # Limpiar buffer si crece demasiado (evita memory leak)
                if len(buffer) > 100:
                    buffer = buffer[-SENSOR_MSG_SIZE:]
                    self.sensor_stats['errors'] += 1
                
            except Exception as e:
                # Error crítico, limpiar buffer
                buffer = bytearray()
                time.sleep(0.01)
            
            # Pequeña pausa para no saturar CPU
            time.sleep(0.005)  # 5ms es suficiente para 200ms de envío

    def get_light_value(self, robot_id):
        """Obtener última lectura de luz de un robot (thread-safe)"""
        with self.light_lock:
            return self.light_readings.get(robot_id, None)

    def get_all_light_values(self):
        """Obtener todas las lecturas de luz (thread-safe)"""
        with self.light_lock:
            return self.light_readings.copy()
    
    def print_sensor_stats(self):
        """Debug: ver estadísticas de recepción"""
        print(f"📊 Sensores recibidos: {self.sensor_stats['received']}, Errores: {self.sensor_stats['errors']}")
    

    def send_command(self, robot_id: int, command: str, speed: int = 600):
        packet = struct.pack('<Bchff', robot_id, command.encode('ascii'), int(speed), 0.0, 0.0)
        self.serial.write(packet)
        self.serial.flush()

    def stop(self, robot_id: int):
        self.send_command(robot_id, 'S', 0)

    def close(self):
        self.running = False
        self.sensor_thread.join(timeout=1)
        self.serial.close()


class PSOParticle:
    """Representa una partícula PSO para cada robot"""
    def __init__(self, robot_id, initial_pos=None):
        self.robot_id = robot_id
        
        # Posición actual (se actualiza del tracker)
        self.position = initial_pos if initial_pos else (0.5, 0.5)
        
        # Velocidad (en espacio normalizado 0-1)
        self.velocity = (random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05))
        
        # Mejor posición personal
        self.pbest_position = self.position
        self.pbest_fitness = float('inf')  # Menor luz = mejor (queremos MINIMIZAR)
        
    def update_fitness(self, light_value):
        """Actualiza pbest si encontró mejor fitness"""
        if light_value is not None and light_value < self.pbest_fitness:
            self.pbest_fitness = light_value
            self.pbest_position = self.position
            return True
        return False

def move_robots_to_targets(robot_configs):
    """
    robot_configs: Lista de diccionarios con ID y Target.
    Ej: [{'id': 4, 'target': (0.9, 0.5)}, {'id': 5, 'target': (0.2, 0.2)}]
    """
    # Configuración del espacio
    WIDTH_CM = 130.0   
    HEIGHT_CM = 70.0
    SCALE_FACTOR = WIDTH_CM / HEIGHT_CM
    
    # Parámetros de movimiento
    ANGLE_TOLERANCE = 18
    DIST_TOLERANCE_CM = 5.0
    TURN_PULSE = 0.12
    MOVE_PULSE = 0.3
    SPEED_ANGLE = 400
    SPEED_LINEAR = 600
    
    # Parámetros de evasión
    SAFE_DISTANCE_CM = 30.0
    REPULSION_STRENGTH = 22.0
    EMERGENCY_DISTANCE_CM = 18.0
    EMERGENCY_STRENGTH = 80.0
    FRONT_MULTIPLIER = 2.0
    RECALC_INTERVAL = 0.15
    
    tracker = ArucoTracker(marker_length=0.075, camera_index=0)
    tracker.start(show_visualization=True)
    controller = SimpleRobotController()
    
    busy_until = {cfg['id']: 0 for cfg in robot_configs}
    last_recalc_time = {cfg['id']: 0 for cfg in robot_configs}
    last_target_angle = {cfg['id']: None for cfg in robot_configs}
    active_robots = {cfg['id'] for cfg in robot_configs}
    last_light_print = 0
    LIGHT_PRINT_INTERVAL = 1.0  # Imprimir cada 1 segundo
    

    try:
        while active_robots:
            markers = tracker.get_all_markers()
            current_time = time.time()
            if current_time - last_light_print >= LIGHT_PRINT_INTERVAL:
                all_lights = controller.get_all_light_values()
                if all_lights:
                    light_str = " | ".join([f"R{rid}:{val}" for rid, val in all_lights.items()])
                    print(f"💡 {light_str}")
                last_light_print = current_time
            for cfg in robot_configs:
                r_id = cfg['id']
                light_val = controller.get_light_value(r_id)
                if light_val is not None:
                    # Aquí puedes usar el valor de luz para PSO
                    # Por ejemplo, almacenarlo o imprimirlo
                    if current_time % 2 < 0.1:  # Imprimir cada 2 segundos
                        # print(f"💡 Robot {r_id}: Luz = {light_val}")
                        continue

                target_x, target_y = cfg['target']
                
                # Convertir target a cm
                TARGET_X_CM = target_x * WIDTH_CM
                TARGET_Y_CM = target_y * HEIGHT_CM

                # 1. Saltamos si el robot ya llegó a su meta
                if r_id not in active_robots:
                    continue

                # 2. No bloqueante: si el robot aún está ejecutando un pulso, pasamos
                if current_time < busy_until[r_id]:
                    continue

                # 3. Verificar si el tracker ve al robot
                if r_id not in markers:
                    continue

                marker = markers[r_id]
                rx = marker.position[0] * WIDTH_CM
                ry = marker.position[1] * HEIGHT_CM
                current_angle = marker.angle % 360
                current_angle_rad = math.radians(current_angle)
                
                # **RECALCULAR SOLO CADA CIERTO INTERVALO**
                if current_time - last_recalc_time[r_id] >= RECALC_INTERVAL:
                    # Normalizar Y
                    ry_normalized = ry * SCALE_FACTOR
                    
                    # 1. Vector atracción
                    dx = rx - TARGET_X_CM 
                    dy = TARGET_Y_CM - ry
                    
                    # 2. Vector repulsión (evitar otros robots)
                    rep_dx = 0
                    rep_dy = 0
                    for oid, om in markers.items():
                        if oid == r_id or oid not in active_robots:
                            continue
                        
                        ox = om.position[0] * WIDTH_CM
                        oy = om.position[1] * HEIGHT_CM
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
                    
                    last_target_angle[r_id] = math.degrees(math.atan2(final_dy, final_dx)) % 360
                    last_recalc_time[r_id] = current_time
                
                # Usar el último ángulo calculado
                if last_target_angle[r_id] is None:
                    continue
                
                target_angle_deg = last_target_angle[r_id]
                angle_error = (target_angle_deg - current_angle + 180) % 360 - 180
                
                # Verificar si llegó al objetivo
                dx_check = rx - TARGET_X_CM 
                dy_check = TARGET_Y_CM - ry
                dist_real = math.hypot(dx_check, dy_check)

                # Lógica de estados
                if dist_real < DIST_TOLERANCE_CM:
                    print(f"🎯 Robot {r_id} llegó a su destino.")
                    controller.stop(r_id)
                    active_robots.remove(r_id)
                    continue
                
                if abs(angle_error) > ANGLE_TOLERANCE:
                    cmd = 'L' if angle_error > 0 else 'R'
                    duration = TURN_PULSE if abs(angle_error) < 45 else TURN_PULSE * 2
                    controller.send_command(r_id, cmd, SPEED_ANGLE)
                    busy_until[r_id] = current_time + duration
                else:
                    controller.send_command(r_id, 'F', SPEED_LINEAR)
                    busy_until[r_id] = current_time + MOVE_PULSE
            
            # Pequeño respiro
            time.sleep(0.01)

    except KeyboardInterrupt:
        for cfg in robot_configs: 
            controller.stop(cfg['id'])
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