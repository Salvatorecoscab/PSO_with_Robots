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
            print(f"✓ connected to  {port}")
            self.light_readings = {}
            self.light_lock = threading.Lock() 
            self.sensor_stats = {'received': 0, 'errors': 0}

            self.running = True
            self.sensor_thread = threading.Thread(target=self._read_sensors, daemon=True)
            self.sensor_thread.start()
            
        except Exception as e:
            print(f"❌ Error connection with serial: {e}")
            exit()
    def _read_sensors(self):
        """Thread that read the sensor data continiously"""
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
        """Get the last lecture of light"""
        with self.light_lock:
            return self.light_readings.get(robot_id, None)

    def get_all_light_values(self):
        """Get all the lectures from light(thread-safe)"""
        with self.light_lock:
            return self.light_readings.copy()
    
    def print_sensor_stats(self):
        """Debug: See the reception"""
        print(f"📊 Sensors reciebed: {self.sensor_stats['received']}, Errors: {self.sensor_stats['errors']}")

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
    """Represents a particle for each robot"""
    def __init__(self, robot_id, initial_pos=None):
        self.robot_id = robot_id
        
        # Actual position
        self.position = initial_pos if initial_pos else (0.5, 0.5)
        
        # Speed in a normilzed space 
        self.velocity = (random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05))
        
        # Best position
        self.pbest_position = self.position
        self.pbest_fitness = float('inf')  
        
    def update_fitness(self, light_value):
        """Update psbest if it finds a best one"""
        if light_value is not None and light_value < self.pbest_fitness:
            self.pbest_fitness = light_value
            self.pbest_position = self.position
            return True
        return False


def pso_follow_light(robot_configs):
    """
    PSO to follow the light
    """
    # Space
    WIDTH_CM = 180.0   
    HEIGHT_CM = 80.0
    SCALE_FACTOR = WIDTH_CM / HEIGHT_CM
    
    # Movement parameters
    ANGLE_TOLERANCE = 18
    DIST_TOLERANCE_CM = 5.0
    TURN_PULSE = 0.12
    MOVE_PULSE = 0.3
    SPEED_ANGLE = 400
    SPEED_LINEAR = 600
    
    # Avoidance parameters
    SAFE_DISTANCE_CM = 30.0
    REPULSION_STRENGTH = 22.0
    EMERGENCY_DISTANCE_CM = 18.0
    EMERGENCY_STRENGTH = 80.0
    FRONT_MULTIPLIER = 2.0
    RECALC_INTERVAL = 0.15
    
    # PSO Paramteres
    W = 0.5          # Inercia
    C1 = 1.5         # Pbest
    C2 = 1.5         # Social component
    MAX_VELOCITY = 0.08  # Max speed
    PSO_UPDATE_INTERVAL = 2.0  # Update pso each 2 seconds
    GBEST_UPDATE_INTERVAL = 1.0  # Update best each second
    
    tracker = ArucoTracker(marker_length=0.075, camera_index=0)
    tracker.start(show_visualization=True)
    controller = SimpleRobotController()
    
    # Init particles
    particles = {cfg['id']: PSOParticle(cfg['id']) for cfg in robot_configs}
    
    # Leader with the best light
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
    
    print("🔍 PSO started...")
    print("⏱️  Press Ctrl+C to stop\n")

    try:
        while True:  
            markers = tracker.get_all_markers()
            current_time = time.time()
            
            # Print light lectures
            if current_time - last_light_print >= LIGHT_PRINT_INTERVAL:
                all_lights = controller.get_all_light_values()
                if all_lights:
                    light_str = " | ".join([f"R{rid}:{val}" for rid, val in sorted(all_lights.items())])
                    leader_str = f"👑 Leader: R{gbest_robot_id} ({gbest_fitness})" if gbest_robot_id else "👑 Leader: N/A"
                    print(f"💡 {light_str} | {leader_str}")
                last_light_print = current_time
            
            # Update Gbest
            if current_time - last_gbest_update >= GBEST_UPDATE_INTERVAL:
                all_lights = controller.get_all_light_values()
                
                if all_lights:
                    # Find the robot with the lowest value
                    min_light_robot = min(all_lights.items(), key=lambda x: x[1])
                    min_robot_id, min_light_value = min_light_robot
                    
                    # Update gbest if it was found
                    if min_light_value < gbest_fitness and min_robot_id in markers:
                        gbest_fitness = min_light_value
                        gbest_robot_id = min_robot_id
                        marker = markers[min_robot_id]
                        gbest_position = (marker.position[0], marker.position[1])
                        print(f"🌟 New Leader: Robot {gbest_robot_id} with = {gbest_fitness}")
                
                last_gbest_update = current_time
            
            for cfg in robot_configs:
                r_id = cfg['id']
                particle = particles[r_id]
                
                # Verify if tracker is seen
                if r_id not in markers:
                    continue
                
                marker = markers[r_id]
                rx = marker.position[0] * WIDTH_CM
                ry = marker.position[1] * HEIGHT_CM
                current_angle = marker.angle % 360
                current_angle_rad = math.radians(current_angle)
                
                # Update position
                particle.position = (marker.position[0], marker.position[1])
                
                # Update personal fitness
                light_val = controller.get_light_value(r_id)
                if light_val is not None:
                    if particle.update_fitness(light_val):
                        print(f"✨ Robot {r_id} mejoró su pbest: {light_val}")
                
                # Update pso speed
                if current_time - last_pso_update[r_id] >= PSO_UPDATE_INTERVAL:
                    r1, r2 = random.random(), random.random()
                    
                    # Inercia
                    vx = W * particle.velocity[0]
                    vy = W * particle.velocity[1]
                    
                    # Cognitive component
                    vx += C1 * r1 * (particle.pbest_position[0] - particle.position[0])
                    vy += C1 * r1 * (particle.pbest_position[1] - particle.position[1])
                    
                    # Social component
                    vx += C2 * r2 * (gbest_position[0] - particle.position[0])
                    vy += C2 * r2 * (gbest_position[1] - particle.position[1])
                    
                    # Limit speed
                    speed = math.hypot(vx, vy)
                    if speed > MAX_VELOCITY:
                        vx = (vx / speed) * MAX_VELOCITY
                        vy = (vy / speed) * MAX_VELOCITY
                    
                    particle.velocity = (vx, vy)
                    last_pso_update[r_id] = current_time
                
                # Calculate position towards objetctive
                target_x = particle.position[0] + particle.velocity[0]
                target_y = particle.position[1] + particle.velocity[1]
                
                # Limit space [0, 1]
                target_x = max(0.05, min(0.95, target_x))
                target_y = max(0.05, min(0.95, target_y))
                
                TARGET_X_CM = target_x * WIDTH_CM
                TARGET_Y_CM = target_y * HEIGHT_CM
                

                if current_time < busy_until[r_id]:
                    continue
                
                # Calculate direction with avoidance
                if current_time - last_recalc_time[r_id] >= RECALC_INTERVAL:
                    ry_normalized = ry * SCALE_FACTOR
                    
                    # Attraction vector towards objective
                    dx = rx - TARGET_X_CM
                    dy = TARGET_Y_CM - ry
                    
                    # Repulsion vector
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
                
                # Verify if it has reached the objective
                dx_check =  rx - TARGET_X_CM
                dy_check = TARGET_Y_CM - ry
                dist_real = math.hypot(dx_check, dy_check)
                
                # Movement logic to send
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
        print("\n🛑 Stoping robots...")
        for cfg in robot_configs: 
            controller.stop(cfg['id'])
    finally:
        tracker.stop()
        controller.close()
        print("✅ Programm finished")


if __name__ == "__main__":
    # Define robots ids
    robots = [
        {'id': 4},
        {'id': 5},
        {'id': 6}
    ]
    
    pso_follow_light(robots)