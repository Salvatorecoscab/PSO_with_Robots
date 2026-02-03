import numpy as np
import random

class RobotParticle:
    def __init__(self, id, x_init, y_init, w=0.5, c1=1.5, c2=2.0):
        self.id = id
        
        # Estado actual (Posición recibida por ArUco)
        self.x = x_init
        self.y = y_init
        
        # Velocidad actual (Vector de movimiento calculado)
        self.vx = 0.0
        self.vy = 0.0
        
        # Mejores registros
        self.pbest_x = x_init
        self.pbest_y = y_init
        self.pbest_luz = -float('inf') # Buscamos maximizar
        
        # Hiperparámetros
        self.w = w   # Inercia
        self.c1 = c1 # Cognitivo (yo)
        self.c2 = c2 # Social (grupo)

    def update_pbest(self, luz_actual):
        """Actualiza el récord personal si la luz actual es mayor."""
        if luz_actual > self.pbest_luz:
            self.pbest_luz = luz_actual
            self.pbest_x = self.x
            self.pbest_y = self.y
            return True
        return False

    def calculate_velocity(self, gbest_x, gbest_y, max_speed=10.0):
        """Aplica la fórmula del PSO para calcular el siguiente movimiento."""
        r1 = random.random()
        r2 = random.random()
        
        # Fórmula PSO: v(t+1) = w*v(t) + c1*r1*(pbest - x) + c2*r2*(gbest - x)
        self.vx = (self.w * self.vx + 
                   self.c1 * r1 * (self.pbest_x - self.x) + 
                   self.c2 * r2 * (gbest_x - self.x))
        
        self.vy = (self.w * self.vy + 
                   self.c1 * r1 * (self.pbest_y - self.y) + 
                   self.c2 * r2 * (gbest_y - self.y))
        
        # Limitación de velocidad (Clamping) para no saturar motores
        speed = np.sqrt(self.vx**2 + self.vy**2)
        if speed > max_speed:
            self.vx = (self.vx / speed) * max_speed
            self.vy = (self.vy / speed) * max_speed

    def get_target_position(self):
        """Retorna a dónde debería moverse el robot en el siguiente paso."""
        return self.x + self.vx, self.y + self.vy