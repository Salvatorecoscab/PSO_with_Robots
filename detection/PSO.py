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
                        robot_id, light_value = struct.unpack('<Bh', buffer[:3])
                        
                        if robot_id in VALID_ROBOT_IDS:
                            if 0 <= light_value <= 1023:
                                with self.light_lock:
                                    self.light_readings[robot_id] = light_value
                                
                                self.sensor_stats['received'] += 1
                                buffer = buffer[SENSOR_MSG_SIZE:]
                            else:
                                buffer = buffer[1:]
                                self.sensor_stats['errors'] += 1
                        else:
                            buffer = buffer[1:]
                            self.sensor_stats['errors'] += 1
                    
                    except struct.error:
                        buffer = buffer[1:]
                        self.sensor_stats['errors'] += 1
                
                if len(buffer) > 100:
                    buffer = buffer[-SENSOR_MSG_SIZE:]
                    self.sensor_stats['errors'] += 1
                
            except Exception as e:
                buffer = bytearray()
                time.sleep(0.01)
            
            time.sleep(0.005)

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


def pso_follow_light(robot_configs):
    """
    PSO para seguir la luz. Los robots buscan la posición con MENOR valor de luz.
    """
    # Configuración del espacio
    WIDTH_CM = 180.0   
    HEIGHT_CM = 80.0
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
    
    # Parámetros PSO
    W = 0.5          # Inercia
    C1 = 1.5         # Componente cognitivo (pbest)
    C2 = 1.5         # Componente social (gbest)
    MAX_VELOCITY = 0.08  # Velocidad máxima en espacio normalizado
    PSO_UPDATE_INTERVAL = 2.0  # Actualizar PSO cada 2 segundos
    GBEST_UPDATE_INTERVAL = 1.0  # Actualizar gbest cada segundo
    
    tracker = ArucoTracker(marker_length=0.075, camera_index=0)
    tracker.start(show_visualization=True)
    controller = SimpleRobotController()
    
    # Inicializar partículas PSO
    particles = {cfg['id']: PSOParticle(cfg['id']) for cfg in robot_configs}
    
    # Global best (el líder con menor luz)
    gbest_position = (0.5, 0.5)
    gbest_fitness = float('inf')
    gbest_robot_id = None
    
    busy_until = {cfg['id']: 0 for cfg in robot_configs}
    last_recalc_time = {cfg['id']: 0 for cfg in robot_configs}
    last_target_angle = {cfg['id']: None for cfg in robot_configs}
    last_pso_update = {cfg['id']: 0 for cfg in robot_configs}
    last_gbest_update = 0
    last_light_print = 0
    LIGHT_PRINT_INTERVAL = 1.0
    
    print("🔍 PSO iniciado - Buscando la menor cantidad de luz...")
    print("🎯 Los robots seguirán al líder (robot con menor lectura de luz)")
    print("⏱️  Presiona Ctrl+C para detener\n")

    try:
        while True:  # Loop infinito - solo se detiene con Ctrl+C
            markers = tracker.get_all_markers()
            current_time = time.time()
            
            # Imprimir lecturas de luz
            if current_time - last_light_print >= LIGHT_PRINT_INTERVAL:
                all_lights = controller.get_all_light_values()
                if all_lights:
                    light_str = " | ".join([f"R{rid}:{val}" for rid, val in sorted(all_lights.items())])
                    leader_str = f"👑 Líder: R{gbest_robot_id} ({gbest_fitness})" if gbest_robot_id else "👑 Líder: N/A"
                    print(f"💡 {light_str} | {leader_str}")
                last_light_print = current_time
            
            # Actualizar GBEST (buscar el robot con menor luz)
            if current_time - last_gbest_update >= GBEST_UPDATE_INTERVAL:
                all_lights = controller.get_all_light_values()
                
                if all_lights:
                    # Encontrar el robot con MENOR luz
                    min_light_robot = min(all_lights.items(), key=lambda x: x[1])
                    min_robot_id, min_light_value = min_light_robot
                    
                    # Actualizar gbest si es mejor
                    if min_light_value < gbest_fitness and min_robot_id in markers:
                        gbest_fitness = min_light_value
                        gbest_robot_id = min_robot_id
                        marker = markers[min_robot_id]
                        gbest_position = (marker.position[0], marker.position[1])
                        print(f"🌟 NUEVO LÍDER: Robot {gbest_robot_id} con luz = {gbest_fitness}")
                
                last_gbest_update = current_time
            
            for cfg in robot_configs:
                r_id = cfg['id']
                particle = particles[r_id]
                
                # Verificar si el tracker ve al robot
                if r_id not in markers:
                    continue
                
                marker = markers[r_id]
                rx = marker.position[0] * WIDTH_CM
                ry = marker.position[1] * HEIGHT_CM
                current_angle = marker.angle % 360
                current_angle_rad = math.radians(current_angle)
                
                # Actualizar posición de la partícula
                particle.position = (marker.position[0], marker.position[1])
                
                # Actualizar fitness personal
                light_val = controller.get_light_value(r_id)
                if light_val is not None:
                    if particle.update_fitness(light_val):
                        print(f"✨ Robot {r_id} mejoró su pbest: {light_val}")
                
                # Actualizar velocidad PSO
                if current_time - last_pso_update[r_id] >= PSO_UPDATE_INTERVAL:
                    r1, r2 = random.random(), random.random()
                    
                    # Componente de inercia
                    vx = W * particle.velocity[0]
                    vy = W * particle.velocity[1]
                    
                    # Componente cognitivo (hacia pbest)
                    vx += C1 * r1 * (particle.pbest_position[0] - particle.position[0])
                    vy += C1 * r1 * (particle.pbest_position[1] - particle.position[1])
                    
                    # Componente social (hacia gbest - el líder)
                    vx += C2 * r2 * (gbest_position[0] - particle.position[0])
                    vy += C2 * r2 * (gbest_position[1] - particle.position[1])
                    
                    # Limitar velocidad
                    speed = math.hypot(vx, vy)
                    if speed > MAX_VELOCITY:
                        vx = (vx / speed) * MAX_VELOCITY
                        vy = (vy / speed) * MAX_VELOCITY
                    
                    particle.velocity = (vx, vy)
                    last_pso_update[r_id] = current_time
                
                # Calcular posición objetivo basada en PSO
                target_x = particle.position[0] + particle.velocity[0]
                target_y = particle.position[1] + particle.velocity[1]
                
                # Limitar al espacio [0, 1]
                target_x = max(0.05, min(0.95, target_x))
                target_y = max(0.05, min(0.95, target_y))
                
                TARGET_X_CM = target_x * WIDTH_CM
                TARGET_Y_CM = target_y * HEIGHT_CM
                
                # No bloqueante
                if current_time < busy_until[r_id]:
                    continue
                
                # Recalcular dirección con evasión de obstáculos
                if current_time - last_recalc_time[r_id] >= RECALC_INTERVAL:
                    ry_normalized = ry * SCALE_FACTOR
                    
                    # Vector atracción hacia objetivo PSO
                    dx = rx - TARGET_X_CM
                    dy = TARGET_Y_CM - ry
                    
                    # Vector repulsión
                    rep_dx = 0
                    rep_dy = 0
                    for oid, om in markers.items():
                        if oid == r_id:
                            continue
                        
                        ox = om.position[0] * WIDTH_CM
                        oy = om.position[1] * HEIGHT_CM
                        oy_normalized = oy * SCALE_FACTOR
                        
                        dist = math.hypot(rx - ox, ry_normalized - oy_normalized)
                        
                        to_obstacle_x = ox - rx
                        to_obstacle_y = oy_normalized - ry_normalized
                        to_obstacle_angle = math.atan2(to_obstacle_y, to_obstacle_x)
                        
                        angle_diff = abs((to_obstacle_angle - current_angle_rad + math.pi) % (2 * math.pi) - math.pi)
                        
                        direction_factor = 1.0
                        if angle_diff < math.radians(60):
                            direction_factor = FRONT_MULTIPLIER
                        
                        if dist < EMERGENCY_DISTANCE_CM:
                            f = (EMERGENCY_DISTANCE_CM - dist) * EMERGENCY_STRENGTH * direction_factor
                            rep_dx += (ox - rx) / dist * f
                            rep_dy += (ry_normalized - oy_normalized) / dist * f
                        elif dist < SAFE_DISTANCE_CM:
                            f = (SAFE_DISTANCE_CM - dist) * REPULSION_STRENGTH * direction_factor
                            rep_dx += (ox - rx) / dist * f
                            rep_dy += (ry_normalized - oy_normalized) / dist * f
                    
                    rep_dy = rep_dy / SCALE_FACTOR
                    
                    final_dx = dx + rep_dx
                    final_dy = dy + rep_dy
                    
                    last_target_angle[r_id] = math.degrees(math.atan2(final_dy, final_dx)) % 360
                    last_recalc_time[r_id] = current_time
                
                if last_target_angle[r_id] is None:
                    continue
                
                target_angle_deg = last_target_angle[r_id]
                angle_error = (target_angle_deg - current_angle + 180) % 360 - 180
                
                # Verificar si llegó cerca del objetivo actual
                dx_check =  rx - TARGET_X_CM
                dy_check = TARGET_Y_CM - ry
                dist_real = math.hypot(dx_check, dy_check)
                
                # Lógica de movimiento
                if abs(angle_error) > ANGLE_TOLERANCE:
                    cmd = 'L' if angle_error > 0 else 'R'
                    duration = TURN_PULSE if abs(angle_error) < 45 else TURN_PULSE * 2
                    controller.send_command(r_id, cmd, SPEED_ANGLE)
                    busy_until[r_id] = current_time + duration
                else:
                    controller.send_command(r_id, 'F', SPEED_LINEAR)
                    busy_until[r_id] = current_time + MOVE_PULSE
            
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n🛑 Deteniendo robots...")
        for cfg in robot_configs: 
            controller.stop(cfg['id'])
    finally:
        tracker.stop()
        controller.close()
        print("✅ Programa finalizado")


if __name__ == "__main__":
    # Define tus robots (sin target fijo, PSO los guiará)
    mis_robots = [
        {'id': 4},
        {'id': 5},
        {'id': 6}
    ]
    
    pso_follow_light(mis_robots)