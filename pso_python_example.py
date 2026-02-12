import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
import matplotlib.patches as mpatches

class PSOSimulation:
    def __init__(self, n_particles=20, dim=2, bounds=(-10, 10)):
        """
        Particle Swarm Optimization Simulation
        
        Parameters:
        - n_particles: número de partículas
        - dim: dimensión del espacio (2D o 3D)
        - bounds: límites del espacio de búsqueda
        """
        self.n_particles = n_particles
        self.dim = dim
        self.bounds = bounds
        
        # Parámetros PSO
        self.w = 0.7  # inerciaxxxxxxxxxxxxxxxx
        self.c1 = 1.5  # coeficiente cognitivo
        self.c2 = 1.5  # coeficiente social
        
        # Inicializar partículas
        self.reset()
        
    def reset(self):
        """Reinicia la simulación"""
        # Posiciones aleatorias
        self.positions = np.random.uniform(
            self.bounds[0], self.bounds[1], 
            (self.n_particles, self.dim)
        )
        
        # Velocidades aleatorias
        self.velocities = np.random.uniform(
            -1, 1, 
            (self.n_particles, self.dim)
        )
        
        # Mejor posición personal de cada partícula
        self.pbest_positions = self.positions.copy()
        self.pbest_values = np.array([self.fitness(p) for p in self.positions])
        
        # Mejor posición global
        best_idx = np.argmin(self.pbest_values)
        self.gbest_position = self.pbest_positions[best_idx].copy()
        self.gbest_value = self.pbest_values[best_idx]
        
        self.iteration = 0
        
    def fitness(self, position):
        """
        Función objetivo (Sphere function)
        Puedes cambiar esta función por cualquier otra
        """
        return np.sum(position**2)
    
    def rastrigin(self, position):
        """Función de Rastrigin (más compleja con múltiples mínimos locales)"""
        A = 10
        n = len(position)
        return A * n + np.sum(position**2 - A * np.cos(2 * np.pi * position))
    
    def ackley(self, position):
        """Función de Ackley"""
        n = len(position)
        sum_sq = np.sum(position**2)
        sum_cos = np.sum(np.cos(2 * np.pi * position))
        return -20 * np.exp(-0.2 * np.sqrt(sum_sq/n)) - np.exp(sum_cos/n) + 20 + np.e
    
    def update(self):
        """Actualiza las partículas un paso"""
        for i in range(self.n_particles):
            # Componentes aleatorios
            r1, r2 = np.random.rand(2)
            
            # Actualizar velocidad
            cognitive = self.c1 * r1 * (self.pbest_positions[i] - self.positions[i])
            social = self.c2 * r2 * (self.gbest_position - self.positions[i])
            self.velocities[i] = self.w * self.velocities[i] + cognitive + social
            
            # Limitar velocidad
            max_vel = 2.0
            self.velocities[i] = np.clip(self.velocities[i], -max_vel, max_vel)
            
            # Actualizar posición
            self.positions[i] += self.velocities[i]
            
            # Mantener dentro de los límites
            self.positions[i] = np.clip(
                self.positions[i], 
                self.bounds[0], 
                self.bounds[1]
            )
            
            # Evaluar fitness
            fitness_value = self.fitness(self.positions[i])
            
            # Actualizar mejor personal
            if fitness_value < self.pbest_values[i]:
                self.pbest_values[i] = fitness_value
                self.pbest_positions[i] = self.positions[i].copy()
                
                # Actualizar mejor global
                if fitness_value < self.gbest_value:
                    self.gbest_value = fitness_value
                    self.gbest_position = self.positions[i].copy()
        
        self.iteration += 1

class PSOVisualizer:
    def __init__(self, pso, resolution=100):
        self.pso = pso
        self.resolution = resolution
        self.running = False
        
        # Crear figura
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        plt.subplots_adjust(bottom=0.25)
        
        # Crear superficie de contorno
        self.create_contour()
        
        # Elementos visuales
        self.particles_scatter = None
        self.pbest_scatter = None
        self.gbest_scatter = None
        self.trails = [[] for _ in range(self.pso.n_particles)]
        self.trail_lines = []
        
        # Inicializar visualización
        self.init_plot()
        
        # Crear controles
        self.create_controls()
        
        # Animación
        self.anim = None
        
    def create_contour(self):
        """Crea el mapa de contorno de la función objetivo"""
        x = np.linspace(self.pso.bounds[0], self.pso.bounds[1], self.resolution)
        y = np.linspace(self.pso.bounds[0], self.pso.bounds[1], self.resolution)
        X, Y = np.meshgrid(x, y)
        
        Z = np.zeros_like(X)
        for i in range(self.resolution):
            for j in range(self.resolution):
                Z[i, j] = self.pso.fitness(np.array([X[i, j], Y[i, j]]))
        
        self.contour = self.ax.contourf(X, Y, Z, levels=30, cmap='viridis', alpha=0.6)
        self.ax.contour(X, Y, Z, levels=15, colors='black', alpha=0.2, linewidths=0.5)
        plt.colorbar(self.contour, ax=self.ax, label='Fitness')
        
    def init_plot(self):
        """Inicializa los elementos del gráfico"""
        # Partículas actuales
        self.particles_scatter = self.ax.scatter(
            self.pso.positions[:, 0], 
            self.pso.positions[:, 1],
            c='red', s=100, alpha=0.8, 
            edgecolors='darkred', linewidths=2,
            label='Partículas', zorder=5
        )
        
        # Mejores posiciones personales
        self.pbest_scatter = self.ax.scatter(
            self.pso.pbest_positions[:, 0],
            self.pso.pbest_positions[:, 1],
            c='blue', s=50, alpha=0.5,
            marker='x', linewidths=2,
            label='Mejor personal', zorder=4
        )
        
        # Mejor posición global
        self.gbest_scatter = self.ax.scatter(
            [self.pso.gbest_position[0]],
            [self.pso.gbest_position[1]],
            c='gold', s=300, alpha=1.0,
            marker='*', edgecolors='orange', linewidths=2,
            label='Mejor global', zorder=6
        )
        
        # Configurar ejes
        self.ax.set_xlim(self.pso.bounds[0], self.pso.bounds[1])
        self.ax.set_ylim(self.pso.bounds[0], self.pso.bounds[1])
        self.ax.set_xlabel('X', fontsize=12)
        self.ax.set_ylabel('Y', fontsize=12)
        self.ax.set_title(f'PSO Simulation - Iteración: 0 | Mejor fitness: {self.pso.gbest_value:.4f}', 
                         fontsize=14, fontweight='bold')
        self.ax.legend(loc='upper right')
        self.ax.grid(True, alpha=0.3)
        
    def create_controls(self):
        """Crea controles interactivos"""
        # Slider para número de partículas
        ax_particles = plt.axes([0.15, 0.15, 0.3, 0.03])
        self.slider_particles = Slider(
            ax_particles, 'Partículas', 
            1, 50, valinit=self.pso.n_particles, valstep=1
        )
        self.slider_particles.on_changed(self.update_particles)
        
        # Slider para inercia
        ax_w = plt.axes([0.15, 0.10, 0.3, 0.03])
        self.slider_w = Slider(
            ax_w, 'Inercia (w)', 
            0.1, 1.5, valinit=self.pso.w
        )
        self.slider_w.on_changed(self.update_w)
        
        # Slider para c1
        ax_c1 = plt.axes([0.15, 0.05, 0.3, 0.03])
        self.slider_c1 = Slider(
            ax_c1, 'Cognitivo (c1)', 
            0.0, 3.0, valinit=self.pso.c1
        )
        self.slider_c1.on_changed(self.update_c1)
        
        # Slider para c2
        ax_c2 = plt.axes([0.60, 0.15, 0.3, 0.03])
        self.slider_c2 = Slider(
            ax_c2, 'Social (c2)', 
            0.0, 3.0, valinit=self.pso.c2
        )
        self.slider_c2.on_changed(self.update_c2)
        
        # Botón Start/Stop
        ax_button = plt.axes([0.60, 0.10, 0.1, 0.04])
        self.button_startstop = Button(ax_button, 'Start')
        self.button_startstop.on_clicked(self.toggle_animation)
        
        # Botón Reset
        ax_reset = plt.axes([0.75, 0.10, 0.1, 0.04])
        self.button_reset = Button(ax_reset, 'Reset')
        self.button_reset.on_clicked(self.reset_simulation)
        
    def update_particles(self, val):
        """Actualiza el número de partículas"""
        n = int(val)
        if n != self.pso.n_particles:
            self.pso.n_particles = n
            self.reset_simulation(None)
    
    def update_w(self, val):
        self.pso.w = val
    
    def update_c1(self, val):
        self.pso.c1 = val
    
    def update_c2(self, val):
        self.pso.c2 = val
    
    def toggle_animation(self, event):
        """Inicia/detiene la animación"""
        if not self.running:
            self.running = True
            self.button_startstop.label.set_text('Stop')
            self.anim = FuncAnimation(
                self.fig, self.animate, 
                interval=50, blit=False, cache_frame_data=False
            )
        else:
            self.running = False
            self.button_startstop.label.set_text('Start')
            if self.anim:
                self.anim.event_source.stop()
    
    def reset_simulation(self, event):
        """Reinicia la simulación"""
        if self.running:
            self.toggle_animation(None)
        
        self.pso.reset()
        self.trails = [[] for _ in range(self.pso.n_particles)]
        
        # Limpiar líneas de trayectoria
        for line in self.trail_lines:
            line.remove()
        self.trail_lines = []
        
        # Actualizar visualización
        self.update_plot()
    
    def animate(self, frame):
        """Función de animación"""
        if self.running:
            # Guardar trayectorias
            for i in range(self.pso.n_particles):
                self.trails[i].append(self.pso.positions[i].copy())
                if len(self.trails[i]) > 20:  # Mantener últimas 20 posiciones
                    self.trails[i].pop(0)
            
            # Actualizar PSO
            self.pso.update()
            
            # Actualizar visualización
            self.update_plot()
    
    def update_plot(self):
        """Actualiza los elementos del gráfico"""
        # Actualizar partículas
        self.particles_scatter.set_offsets(self.pso.positions)
        
        # Actualizar pbest
        self.pbest_scatter.set_offsets(self.pso.pbest_positions)
        
        # Actualizar gbest
        self.gbest_scatter.set_offsets([self.pso.gbest_position])
        
        # Actualizar trayectorias
        for line in self.trail_lines:
            line.remove()
        self.trail_lines = []
        
        for trail in self.trails:
            if len(trail) > 1:
                trail_array = np.array(trail)
                line, = self.ax.plot(
                    trail_array[:, 0], trail_array[:, 1],
                    'r-', alpha=0.3, linewidth=1, zorder=2
                )
                self.trail_lines.append(line)
        
        # Actualizar título
        self.ax.set_title(
            f'PSO Simulation - Iteración: {self.pso.iteration} | Mejor fitness: {self.pso.gbest_value:.6f}',
            fontsize=14, fontweight='bold'
        )
        
        self.fig.canvas.draw_idle()
    
    def show(self):
        """Muestra la visualización"""
        plt.show()

# Ejecutar simulación
if __name__ == "__main__":
    # Crear simulación PSO
    pso = PSOSimulation(n_particles=20, dim=2, bounds=(-10, 10))
    
    # Crear visualizador
    visualizer = PSOVisualizer(pso)
    
    print("=" * 60)
    print("SIMULACIÓN PSO - Particle Swarm Optimization")
    print("=" * 60)
    print("\nControles:")
    print("  • Partículas: Ajusta el número de partículas (1-50)")
    print("  • Inercia (w): Controla la influencia de la velocidad previa")
    print("  • Cognitivo (c1): Atracción hacia mejor posición personal")
    print("  • Social (c2): Atracción hacia mejor posición global")
    print("  • Start/Stop: Inicia/detiene la animación")
    print("  • Reset: Reinicia la simulación")
    print("\nLeyenda:")
    print("  🔴 Círculos rojos: Posición actual de partículas")
    print("  ✖️  Cruces azules: Mejor posición personal (pbest)")
    print("  ⭐ Estrella dorada: Mejor posición global (gbest)")
    print("  📈 Fondo: Mapa de contorno de la función objetivo")
    print("=" * 60)
    
    visualizer.show()