import math
import pygame
import random
import time
import numpy as np


def optimize(velocity, angle, curr_angle):
    target = angle
    a = abs(angle) % 360 * (-1 if angle < 0 else 1)
    ca = abs(curr_angle) % 360 * (-1 if curr_angle < 0 else 1)
    diff = a - ca
    v = velocity
    if abs(diff) > 90 and abs(diff) < 270:
        b = 180 - diff
        a = ca - b
        angle = curr_angle - b
        v *= -1
    elif abs(diff) >= 270:
        if diff < 0:
            angle = curr_angle + (360 + diff)
        else:
            angle = curr_angle - (360 - diff)
    else:
        angle = curr_angle + diff
    # print(f"v:{v} t_a:{a} next_angle:{angle} diff:{diff} target:{target} curr_angle:{curr_angle}")
    return v, angle


class SwerveModule:
    module_location: tuple
    center: tuple
    radius: float
    draw_radius: int = 30

    kP: float = 0.01
    kI: float = 1e-6
    kD: float = 0
    kP_v: float = 1e-2

    MAX_VELOCIY: int = 2
    curr_velocity: float = 0
    target_velocity: float = 0

    curr_angle: float = 0
    target_angle: float = 0

    curr_dthetadt = 0
    curr_error = 0

    def __init__(self, screen, module_location, center, radius):
        self.screen = screen
        self.module_location = module_location
        self.center = center
        self.radius = radius
        self.r = np.array([module_location[0]-center[0],
                          center[1]-module_location[1], 0])
    
    def coord(self, x, y):
        return (self.module_location[0] + x, self.module_location[1] - y)

    def get_circle_point(self, angle):
        return self.draw_radius*math.cos(math.radians(angle)), self.draw_radius*math.sin(math.radians(angle))

    def get_circle_point_large(self, angle):
        return 1.5*self.draw_radius*math.cos(math.radians(angle)), 1.5*self.draw_radius*math.sin(math.radians(angle))

    def draw_arrow(self, p1, l, a, color=(100,0,255)):
        if l > 0:
            p2 = self.coord(l*math.cos(math.radians(a)), l*math.sin(math.radians(a)))
            pygame.draw.line(self.screen, color, p1, p2)
            p3 = self.coord(l*math.cos(math.radians(a-2)), l*math.sin(math.radians(a-2)))
            p4 = self.coord((l+10)*math.cos(math.radians(a)), (l+10)*math.sin(math.radians(a)))
            p5 = self.coord(l*math.cos(math.radians(a+2)), l*math.sin(math.radians(a+2)))
            pygame.draw.polygon(self.screen, color, [p2, p3, p4, p5])

    def update(self, target_angular_velocity, target_velocity, dt):
        self.kinematics(target_angular_velocity, target_velocity)
        r_x, r_y = self.get_circle_point_large(self.curr_angle - 20)
        r1_x, r1_y = self.get_circle_point_large(self.curr_angle + 20)
        r2_x, r2_y = self.get_circle_point_large(self.curr_angle + 180 - 20)
        r3_x, r3_y = self.get_circle_point_large(self.curr_angle + 180 + 20)
        c1 = self.coord(r_x, r_y)
        c2 = self.coord(r1_x, r1_y)
        c3 = self.coord(r2_x, r2_y)
        c4 = self.coord(r3_x, r3_y)
        pygame.draw.polygon(self.screen, (0,0,0),[c1,c2,c3,c4])

        pygame.draw.circle(self.screen, (50,50,50), self.coord(0,0), self.draw_radius)
        
        t_x, t_y = self.get_circle_point(self.target_angle)
        a_x, a_y = self.get_circle_point(self.target_angle + 90)
        a1_x, a1_y = self.get_circle_point(self.target_angle + 180)
        a2_x, a2_y = self.get_circle_point(self.target_angle + 270)
        c_x, c_y = self.get_circle_point(self.curr_angle)

        v, new_angle = optimize(self.target_velocity, self.target_angle, self.curr_angle)
        self.target_velocity = v

        pygame.draw.line(self.screen, (255, 0, 0), self.coord(0,0), self.coord(t_x, t_y), width=3)
        pygame.draw.line(self.screen, (100, 100, 100), self.coord(0,0), self.coord(a_x, a_y), width=3)
        pygame.draw.line(self.screen, (100, 100, 100), self.coord(0,0), self.coord(a1_x, a1_y), width=3)
        pygame.draw.line(self.screen, (100, 100, 100), self.coord(0,0), self.coord(a2_x, a2_y), width=3)

        pygame.draw.line(self.screen, (0, 255, (255 if v < 0 else 0)), self.coord(0,0), self.coord(c_x, c_y), width=3)
        self.draw_arrow(self.coord(-c_x, -c_y) if self.curr_velocity < 0 else self.coord(c_x, c_y), (abs(self.curr_velocity)/self.MAX_VELOCIY)*2*self.draw_radius, self.curr_angle + (180 if v < 0 else 0))

        diff = (abs(self.curr_angle - new_angle) % 360)
        past_angle = self.curr_angle
        dthetadt = diff / dt
        self.curr_error += diff * dt
        if (new_angle < self.curr_angle and diff > 0.005):
            self.curr_angle -= self.kP * diff + self.kI * self.curr_error + self.kD * dthetadt
        elif (new_angle > self.curr_angle and diff > 0.005):
            self.curr_angle += self.kP * diff + self.kI * self.curr_error + self.kD * dthetadt
        
        v_diff = self.target_velocity - self.curr_velocity
        self.curr_velocity += self.kP_v * v_diff
        dthetadt = (self.curr_angle - past_angle) / dt
        domegadt = (dthetadt - self.curr_dthetadt) / dt
        self.curr_dthetadt = dthetadt


    def kinematics(self, target_angular_velocity, target_velocity):
        w = np.array([0, 0, target_angular_velocity])
        v_rot = np.cross(w, self.r)
        v_net = np.array([target_velocity[0] + v_rot[0],
                         target_velocity[1] + v_rot[1]])
        # if v_net[0] == 0:
        #     if v_net[1] > 0:
        #         self.target_angle = 90
        #     else:
        #         self.target_angle = 270
        # elif v_net[0] < 0:
        #     self.target_angle = math.degrees(math.atan(v_net[1]/v_net[0])) + 180.0
        # else:
        #     self.target_angle = math.degrees(math.atan(v_net[1]/v_net[0]))
        self.target_angle = self.get_angle_from_velocity(v_net)
        self.target_velocity = math.sqrt(v_net[0]**2 + v_net[1]**2)

        v_rot_mag = math.sqrt(v_rot[0]**2 + v_rot[1]**2)
        v_rot_scaled_mag = abs(v_rot_mag) * 2 * self.draw_radius / self.MAX_VELOCIY
        v_rot_angle = self.get_angle_from_velocity(v_rot)
        self.draw_arrow(self.module_location, v_rot_scaled_mag, v_rot_angle, color=(0,100,255))

        v_t = target_velocity
        v_t_mag = math.sqrt(v_t[0]**2 + v_t[1]**2)
        v_t_scaled_mag = abs(v_t_mag) * 2 * self.draw_radius / self.MAX_VELOCIY
        v_t_angle = self.get_angle_from_velocity(v_t)
        self.draw_arrow(self.module_location, v_t_scaled_mag, v_t_angle, color=(0,255,100))
    

    def get_angle_from_velocity(self, v):
        if v[0] == 0:
            if v[1] > 0:
                return 90
            else:
                return 270
        elif v[0] < 0:
            return math.degrees(math.atan(v[1]/v[0])) + 180.0
        else:
            return math.degrees(math.atan(v[1]/v[0]))


def main():
    w = 1200
    h = 650
    r = 100

    pygame.init()
    pygame.joystick.init()

    joystick = pygame.joystick.Joystick(0)

    screen = pygame.display.set_mode([w, h])
    running = True

    disp_x = 100
    disp_y = 100

    module1 = SwerveModule(screen, (w//2 + disp_x, h//2 - disp_y), (w//2, h//2), math.sqrt(2 * (disp_x**2)))
    module2 = SwerveModule(screen, (w//2 - disp_x, h//2 - disp_y), (w//2, h//2), math.sqrt(2 * (disp_x**2)))
    module3 = SwerveModule(screen, (w//2 + disp_x, h//2 + disp_y), (w//2, h//2), math.sqrt(2 * (disp_x**2)))
    module4 = SwerveModule(screen, (w//2 - disp_x, h//2 + disp_y), (w//2, h//2), math.sqrt(2 * (disp_x**2)))

    p1 = module1.module_location
    p2 = module2.module_location
    p3 = module3.module_location
    p4 = module4.module_location

    past_time = time.time()

    v_x = 0.1
    v_y = 0.1
    v_w = 0.1

    font = pygame.font.Font(None, 24)

    def render_text(screen, text, x, y):
        text = font.render(text, True, (0,0,0), (255,255,255))
        text_rect = text.get_rect()
        text_rect.center = (x, y)
        screen.blit(text, text_rect)

    while running:
        for event in pygame.event.get():    
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RIGHT:
                    v_w += 0.1
                if event.key == pygame.K_LEFT:
                    v_w -= 0.1
                if event.key == pygame.K_w:
                    v_x += 0.1
                if event.key == pygame.K_s:
                    v_x -= 0.1
                if event.key == pygame.K_a:
                    v_y -= 0.1
                if event.key == pygame.K_d:
                    v_y += 0.1
                # print(f"v_x: {v_x}, v_y:{v_y}, v_w:{v_w}")

        screen.fill((255, 255, 255))

        v_x = 5*(joystick.get_axis(0) if abs(joystick.get_axis(0)) > 0.2 else 0)
        v_y = 5*(joystick.get_axis(1) if abs(joystick.get_axis(1)) > 0.2 else 0)
        v_w = 0.1*joystick.get_axis(2) if abs(joystick.get_axis(2)) > 0.2 else 0     

        # v_x = 5*(math.atan(3*joystick.get_axis(0))/(math.pi / 2.0) if abs(joystick.get_axis(0)) > 0.2 else 0)
        # v_y = 5*(math.atan(3*joystick.get_axis(1))/(math.pi / 2.0) if abs(joystick.get_axis(1)) > 0.2 else 0)
        # v_w = 0.1*math.atan(3*joystick.get_axis(2))/(math.pi / 2.0) if abs(joystick.get_axis(2)) > 0.2 else 0     

        render_text(screen, f"v_x: {v_x:.2f}", 50,100)
        render_text(screen, f"v_y: {v_y:.2f}", 50,140)
        render_text(screen, f"v_w: {v_w:.2f}", 50,180)

        curr_time = time.time()
        dt = curr_time - past_time
        if dt <= 0:
            dt = 1e-10
        
        pygame.draw.polygon(screen, (100,100,100),[p1,p3,p4,p2])

        module1.update(v_w, (v_y, v_x), dt)
        module2.update(v_w, (v_y, v_x), dt)
        module3.update(v_w, (v_y, v_x), dt)
        module4.update(v_w, (v_y, v_x), dt)


        past_time = curr_time
        pygame.display.flip()

    pygame.quit()


main()
