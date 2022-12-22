import math
import pygame
import random
import time

CURR_ANGLE = 0 + 360 # degrees
VELOCITY = 5

def optimize(velocity, angle, curr_angle):
    target = angle
    a = abs(angle) % 360
    ca = abs(curr_angle) % 360 * (-1 if curr_angle < 0 else 1)
    diff = a - ca
    v = velocity
    if abs(diff) > 90 and abs(diff) < 270:
        b = 180 - diff
        a = ca - b
        angle = curr_angle - b
        v *= -1
    else:
        angle = curr_angle + diff
    # print(f"v:{v} a:{a} angle:{angle} diff:{diff} target:{target}")
    return v, angle


# class SwerveModule:


def run_window():
    # Screen Constants
    w = 1000
    h = 600
    r = 100

    def coord(x, y):
        return (w//2 + x, h//2 - y)
    def get_circle_point(angle):
        return r*math.cos(math.radians(angle)), r*math.sin(math.radians(angle))
    def get_circle_point_large(angle):
        return 1.5*r*math.cos(math.radians(angle)), 1.5*r*math.sin(math.radians(angle))
    def render_text(screen, text, x, y):
        text = font.render(text, True, (0,0,0), (255,255,255))
        text_rect = text.get_rect()
        text_rect.center = (coord(x, y))
        screen.blit(text, text_rect)
    def draw_arrow(screen, p1, l, a):
        p2 = coord(l*math.cos(math.radians(a)), l*math.sin(math.radians(a)))
        pygame.draw.line(screen, (100,0,255), p1, p2)
        p3 = coord(l*math.cos(math.radians(a-2)), l*math.sin(math.radians(a-2)))
        p4 = coord((l+10)*math.cos(math.radians(a)), (l+10)*math.sin(math.radians(a)))
        p5 = coord(l*math.cos(math.radians(a+2)), l*math.sin(math.radians(a+2)))
        pygame.draw.polygon(screen, (100,0,255), [p2, p3, p4, p5])

    pygame.init()
    screen = pygame.display.set_mode([w, h])
    running = True
    font = pygame.font.Font(None, 24)

    target_angle = 90
    curr_angle = 0
    curr_time = time.time()
    curr_dthetadt = 0
    curr_error = 0
    kP = 0.01
    kI = 1e-6#0.000001 1e-40
    kD = 0#0.00001 #1e-40

    curr_velocity = VELOCITY
    target_velocity = VELOCITY
    kP_v = 1e-2

    while running:
        for event in pygame.event.get():    
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RIGHT:
                    target_angle = random.randrange(-360, 360)

        screen.fill((255, 255, 255))

        r_x, r_y = get_circle_point_large(curr_angle - 20)
        r1_x, r1_y = get_circle_point_large(curr_angle + 20)
        r2_x, r2_y = get_circle_point_large(curr_angle + 180 - 20)
        r3_x, r3_y = get_circle_point_large(curr_angle + 180 + 20)
        c1 = coord(r_x, r_y)
        c2 = coord(r1_x, r1_y)
        c3 = coord(r2_x, r2_y)
        c4 = coord(r3_x, r3_y)
        pygame.draw.polygon(screen, (0,0,0),[c1,c2,c3,c4])

        pygame.draw.circle(screen, (50,50,50), coord(0,0), r)
        
        t_x, t_y = get_circle_point(target_angle)
        a_x, a_y = get_circle_point(target_angle + 90)
        a1_x, a1_y = get_circle_point(target_angle + 180)
        a2_x, a2_y = get_circle_point(target_angle + 270)
        c_x, c_y = get_circle_point(curr_angle)

        v, new_angle = optimize(VELOCITY, target_angle + 360 if target_angle < 0 else target_angle, curr_angle)
        target_velocity = v

        pygame.draw.line(screen, (255, 0, 0), coord(0,0), coord(t_x, t_y), width=3)
        pygame.draw.line(screen, (100, 100, 100), coord(0,0), coord(a_x, a_y), width=3)
        pygame.draw.line(screen, (100, 100, 100), coord(0,0), coord(a1_x, a1_y), width=3)
        pygame.draw.line(screen, (100, 100, 100), coord(0,0), coord(a2_x, a2_y), width=3)

        pygame.draw.line(screen, (0, 255, (255 if v < 0 else 0)), coord(0,0), coord(c_x, c_y), width=3)
        draw_arrow(screen, coord(-c_x, -c_y) if curr_velocity < 0 else coord(c_x, c_y), (abs(curr_velocity)/VELOCITY)*2*r, curr_angle + (180 if curr_velocity < 0 else 0))

        diff = (abs(curr_angle - new_angle) % 360)
        dt = time.time() - curr_time + 1e-10
        curr_time = time.time()
        past_angle  = curr_angle
        dthetadt = diff / dt
        curr_error += diff * dt
        if (new_angle < curr_angle and diff > 0.005):
            curr_angle -= kP * diff + kI * curr_error + kD * dthetadt
        elif (new_angle > curr_angle and diff > 0.005):
            curr_angle += kP * diff + kI * curr_error + kD * dthetadt
        
        v_diff = target_velocity - curr_velocity
        curr_velocity += kP_v * v_diff
        # if abs(v_diff) > 0.1:
        #     curr_velocity += kP_v * v_diff

        # else:
        #     target_angle = random.randrange(-360, 360)
        dthetadt = (curr_angle - past_angle) / dt
        domegadt = (dthetadt - curr_dthetadt) / dt
        curr_dthetadt = dthetadt

        render_text(screen, f"v: {curr_velocity:.3f} m/s", -255, 265)
        render_text(screen, f"adjusted target angle a: {new_angle:.2f} deg", -255, 245)
        render_text(screen, f"current angle c: {curr_angle:.2f} deg", -255, 225)
        render_text(screen, f"target angle t: {target_angle} deg", -255, 205)
        render_text(screen, f"w: {dthetadt:.2f} degrees/s", -255, 185)
        render_text(screen, f"alpha: {domegadt:.2f} degrees/s^2", -255, 165)

        pygame.display.flip()

    pygame.quit()


def main():
    run_window()
    # print(f"CURR_ANGLE: {CURR_ANGLE} ({CURR_ANGLE%360} degrees)")
    # optimize(VELOCITY, 190)
    # optimize(VELOCITY, 90)
    # optimize(VELOCITY, 70)
    # optimize(VELOCITY, -70)
    # optimize(VELOCITY, 267)


main()