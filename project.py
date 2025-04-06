import pygame
import numpy as np
from scipy.integrate import ode
import math

pygame.init()

# Set the dimensions of the screen
WIDTH, HEIGHT = 1600, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Aircraft Takeoff Simulation")

WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
LIGHT_BLUE = (166, 232, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)

cam_offset = 10
zoom = 0.33 # scale factor for simulation coords to screen coords

takeoff_distance = -1
takeoff_velocity = -1
takeoff_angle = -1

class AircraftSimulation:
    def __init__(self):
        # define aircraft variables below
        self.mass = 78000.0         # Kg
        self.thrust = 240000.0      # Newtons
        self.air_density = 1.225    # kg/m^3
        self.wing_area = 122.4      # m^2
        self.g = 9.81               # m/s^2
        self.pitch_angle = 0.0      # degrees
        self.in_air = False
        self.flaps_lowered = False
        self.gear_lowered = True
        self.aoa = 0.0              # degrees
        self.trail = []

        self.state = np.array([0.0, 0.0, 
                               0.0, 0.0]) #(x, y, vx, vy)

        self.solver = ode(self.f) 
        self.solver.set_integrator('dop853')  
        self.solver.set_initial_value(self.state, 0) 
        self.time = 0.0 

    def f(self, t, state):
        x, y, vx, vy = state

        if vy == 0:
            self.aoa = 0
        else:
            self.aoa =  self.pitch_angle - math.degrees(math.atan(vy/vx))
        print("aoa: ", self.aoa)
        if int(self.time / 0.01) % 50 == 0:
            self.trail.append((x, y))

        v = math.sqrt(vx**2 + vy**2)

        drag = self.get_drag_coeff() * 0.5 * self.air_density * self.wing_area * v**2 
        
        lift = self.get_lift_coeff() * 0.5 * self.air_density * self.wing_area * v**2
        
        ax = (-lift*math.sin(math.radians(self.pitch_angle)) + self.thrust*math.cos(math.radians(self.pitch_angle)) - drag*math.cos(math.radians(self.pitch_angle))) / self.mass 
        ay = (lift*math.cos(math.radians(self.pitch_angle)) + self.thrust*math.sin(math.radians(self.pitch_angle)) - drag*math.sin(math.radians(self.pitch_angle))) / self.mass - self.g 
        
        if vy < 0 and y <= 0:  # prevent plane from sinking through ground
            vy = 0
            if ay < 0:
                ay = 0

        return [vx, vy, ax, ay]

    def update(self, dt):
        self.solver.integrate(self.time + dt)
        self.state = self.solver.y
        self.time += dt

    def get_state(self):
        return self.state
    
    def toggle_flaps(self):
        self.flaps_lowered = not self.flaps_lowered

    def toggle_gear(self):
        if self.gear_lowered:
            self.gear_lowered = False

    def get_drag_coeff(self):
        drag_coeff = 0.02
        drag_coeff += self.aoa*0.001
        if self.flaps_lowered:
            drag_coeff += 0.04
        if self.gear_lowered:
            drag_coeff += 0.06
        return drag_coeff
    
    def get_lift_coeff(self):
        lift_coeff = 0.5
        lift_coeff += self.aoa * 0.08
        if self.flaps_lowered:
            lift_coeff += 0.6
        return lift_coeff

    def change_pitch(self, increasing):
        if increasing:
            self.pitch_angle += 5
            self.pitch_angle = min(self.pitch_angle, 20)
        else:
            self.pitch_angle -= 5
            self.pitch_angle = max(self.pitch_angle, 0)

def main():
    simulation = AircraftSimulation()

    running = True
    clock = pygame.time.Clock()

    plane = pygame.image.load("plane.png")
    plane = pygame.transform.scale(plane, (100, 40))
    rotated_plane = plane

    while running:
        screen.fill(LIGHT_BLUE)  # Background color

        simulation.update(0.01) 

        x, y, vx, vy = simulation.get_state()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_w:
                simulation.change_pitch(True)
            if event.type == pygame.KEYDOWN and event.key == pygame.K_s:
                simulation.change_pitch(False)
            if event.type == pygame.KEYDOWN and event.key == pygame.K_f:
                simulation.toggle_flaps()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_g and y > 0:
                simulation.toggle_gear()

        if vy > 0 and not simulation.in_air:
            simulation.in_air = True
            takeoff_distance = x
            takeoff_velocity = vx
            takeoff_angle = simulation.pitch_angle

        aircraft_rect = pygame.Rect(x*zoom, HEIGHT - y*zoom - 40 - cam_offset, 50, 30)  # flipping y-axis
        rotated_plane = pygame.transform.rotate(plane, simulation.pitch_angle)
        screen.blit(rotated_plane, aircraft_rect)

        pygame.draw.rect(screen, BLACK, pygame.Rect(0, HEIGHT - cam_offset, WIDTH, 5)) # draw runway
        pygame.draw.rect(screen, (59, 130, 68), pygame.Rect(0, HEIGHT - cam_offset + 5, WIDTH, 20))

        for i in range(len(simulation.trail) - 1):
            start = simulation.trail[i]
            end = simulation.trail[i + 1]
            pygame.draw.line(screen, BLUE, (start[0] * zoom, HEIGHT - start[1] * zoom - cam_offset - 10), (end[0] * zoom, HEIGHT - end[1] * zoom - cam_offset - 10), 2)


        font = pygame.font.SysFont("Arial", 24)
        velocity_text = font.render(f"v: {math.sqrt(vx**2 + vy**2):.2f} m/s", True, BLACK)
        position_text = font.render(f"Takeoff Distance: {x:.2f} m", True, BLACK)
        altitude_text = font.render(f"Altitude: {y:.2f} m", True, BLACK)
        degree_text = font.render(f"pitch: {simulation.pitch_angle:.2f}°", True, BLACK)
        aoa_text = font.render(f"Angle of Attack: {simulation.aoa:.2f}°", True, BLACK)
        flaps_text = font.render(f"Flaps Lowered: {simulation.flaps_lowered}", True, BLACK)
        gear_text = font.render(f"Gear Lowered: {simulation.gear_lowered}", True, BLACK)


        screen.blit(velocity_text, (10, 10))
        screen.blit(position_text, (10, 40))
        screen.blit(altitude_text, (10, 70))
        screen.blit(degree_text, (10, 100))
        screen.blit(aoa_text, (10, 130))
        screen.blit(flaps_text, (10, 160))
        screen.blit(gear_text, (10, 190))



        if simulation.in_air:
            takeoff_dist_text = font.render(f"Takeoff Distance: {takeoff_distance:.2f} m", True, BLACK)
            takeoff_vel_text = font.render(f"Takeoff Speed: {takeoff_velocity:.2f} m/s", True, BLACK)
            takeoff_angle_text = font.render(f"Takeoff Pitch: {takeoff_angle:.2f}°", True, BLACK)

            screen.blit(takeoff_dist_text, (1300, 10))
            screen.blit(takeoff_vel_text, (1300, 40))
            screen.blit(takeoff_angle_text, (1300, 70))

        if y >= 900:
            pygame.quit()

        pygame.display.flip()  

        clock.tick(60)  

    pygame.quit()

if __name__ == '__main__':
    main()