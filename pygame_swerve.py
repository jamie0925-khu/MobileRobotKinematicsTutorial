import pygame
import numpy as np
import math
import sys

# parameters
L = 0.625  
W = 0.525   
SPEED = 1.5
TURN_SPEED = 1.5
BODY_W = 120   
BODY_H = 80    
WIDTH, HEIGHT = 1000, 700

# 초기 상태
robot_x = WIDTH // 2
robot_y = HEIGHT // 2
robot_theta = 0.0

def compute_swerve(vx, vy, omega):
    LX = L / 2
    WY = W / 2
    # 속도 구성요소 
    v1x = vx + omega * WY
    v1y = vy + omega * LX

    v2x = vx - omega * WY
    v2y = vy + omega * LX

    v3x = vx - omega * WY
    v3y = vy - omega * LX

    v4x = vx + omega * WY
    v4y = vy - omega * LX

    wheels = [(v1x, v1y), (v2x, v2y), (v3x, v3y), (v4x, v4y)]

    results = []
    speeds = []

    for vx_i, vy_i in wheels:
        speed = math.sqrt(vx_i**2 + vy_i**2)
        angle = math.atan2(vy_i, vx_i)
        results.append([angle, speed])
        speeds.append(speed)

    # normalize (논문에서 사용)
    vmax = max(speeds)
    if vmax > 1:
        for r in results:
            r[1] /= vmax

    return results


def draw_robot(x, y, theta, wheel_states):
    rect_surface = pygame.Surface((BODY_W, BODY_H), pygame.SRCALPHA)
    rect_surface.fill((70, 70, 70, 180)) 

    rotated_body = pygame.transform.rotate(rect_surface, -math.degrees(theta))
    body_rect = rotated_body.get_rect(center=(x, y))
    screen.blit(rotated_body, body_rect)

    front_len = BODY_H // 2
    fx = x + math.cos(theta) * front_len
    fy = y + math.sin(theta) * front_len
    pygame.draw.line(screen, (255, 0, 0), (x, y), (fx, fy), 4)

    offsets = [
        (+50, -25),   # Front Right
        (-50, -25),   # Front Left
        (-50, +25),   # Rear Left
        (+50, +25)    # Rear Right
    ]

    for idx, (ox, oy) in enumerate(offsets):
        wx = x + math.cos(theta) * ox - math.sin(theta) * oy
        wy = y + math.sin(theta) * ox + math.cos(theta) * oy

        ang, spd = wheel_states[idx]

        # 바퀴 방향 선
        wheel_len = 35
        dx = math.cos(ang + theta) * wheel_len
        dy = math.sin(ang + theta) * wheel_len

        pygame.draw.line(screen, (0, 200, 255), (wx, wy), (wx + dx, wy + dy), 4)
        pygame.draw.circle(screen, (255, 255, 255), (int(wx), int(wy)), 8)

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Swerve Drive Simulator (Based on Paper)")
clock = pygame.time.Clock()

running = True
while running:
    dt = clock.tick(60) / 1000.0
    screen.fill((25, 25, 25))

    # 키보드 조작
    keys = pygame.key.get_pressed()
    vx = vy = omega = 0

    if keys[pygame.K_w]:
        vx = SPEED
    if keys[pygame.K_s]:
        vx = -SPEED
    if keys[pygame.K_a]:
        vy = -SPEED
    if keys[pygame.K_d]:
        vy = SPEED
    if keys[pygame.K_q]:
        omega = -TURN_SPEED
    if keys[pygame.K_e]:
        omega = TURN_SPEED

    wheel_states = compute_swerve(vx, vy, omega)

    # 로봇 pose update
    robot_x += (vx * math.cos(robot_theta) - vy * math.sin(robot_theta)) * 80 * dt
    robot_y += (vx * math.sin(robot_theta) + vy * math.cos(robot_theta)) * 80 * dt
    robot_theta += omega * dt

    draw_robot(robot_x, robot_y, robot_theta, wheel_states)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.update()

pygame.quit()
sys.exit()
