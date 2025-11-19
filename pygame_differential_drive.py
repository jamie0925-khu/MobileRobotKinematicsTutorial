import pygame
import math
import numpy as np

# Parameters
r = 0.05    # wheel radius [m]
a = 0.2     # half wheel spacing [m]
dt = 0.02
scale = 200
WIDTH, HEIGHT = 900, 700

x, y, theta = 0.0, 0.0, 0.0
path = []
phi_r_dot = 0.0
phi_l_dot = 0.0

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# 좌표변환 
def world_to_screen(wx, wy):
    sx = WIDTH//2 + int(wx * scale)
    sy = HEIGHT//2 - int(wy * scale)
    return sx, sy

# 로봇 생성
def draw_robot(surf, x, y, theta):
    cx, cy = world_to_screen(x, y)

    # 로봇 차체
    pygame.draw.circle(surf, (255,180,90), (cx, cy), 12)
    hx = x + 0.12 * math.cos(theta)
    hy = y + 0.12 * math.sin(theta)
    hx_px, hy_px = world_to_screen(hx, hy)
    pygame.draw.line(surf, (255,255,150), (cx,cy), (hx_px,hy_px), 3)

    # 로봇 좌우 방향 벡터 (theta에 수직)
    side_x = -math.sin(theta)
    side_y =  math.cos(theta)

    # 왼쪽 바퀴 좌표
    wl_x = x + a * side_x
    wl_y = y + a * side_y
    wl_px, wl_py = world_to_screen(wl_x, wl_y)

    # 오른쪽 바퀴 좌표
    wr_x = x - a * side_x
    wr_y = y - a * side_y
    wr_px, wr_py = world_to_screen(wr_x, wr_y)

    # 바퀴
    pygame.draw.rect(surf, (200,200,200), (wl_px-6, wl_py-3, 12, 6))
    pygame.draw.rect(surf, (200,200,200), (wr_px-6, wr_py-3, 12, 6))


# 시나리오 선택 (제공된 자료기반)
def select_scenario(num): 
    #바퀴 각속도 (오른쪽, 왼쪽) (rad/s) 
    if num == 1:    
        phi_r_dot = 2.0 
        phi_l_dot = 2.0 
        return phi_r_dot, phi_l_dot 
    
    if num == 2: 
        phi_r_dot = -2.0 
        phi_l_dot = -2.0 
        return phi_r_dot, phi_l_dot
    
    if num == 3: 
        phi_r_dot = 4.0 
        phi_l_dot = 2.0 
        return phi_r_dot, phi_l_dot 
    
    if num == 4: 
        phi_r_dot = 2.0 
        phi_l_dot = 4.0 
        return phi_r_dot, phi_l_dot 
    
    if num == 5: 
        phi_r_dot = 2.0 
        phi_l_dot = -2.0 
        return phi_r_dot, phi_l_dot

# 메인 루프
running = True
while running:
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False

    # 시나리오 선택
    phi_r_dot, phi_l_dot = select_scenario(5)


    # 자코비안
    J = np.array([
        [(r/2)*math.cos(theta), (r/2)*math.cos(theta)],
        [(r/2)*math.sin(theta), (r/2)*math.sin(theta)],
        [(r/(2*a)),             -(r/(2*a))]
    ])
    
    dq = np.array([phi_r_dot, phi_l_dot])
    dx, dy, dtheta = J @ dq

    x += dx*dt
    y += dy*dt
    theta += dtheta*dt

    # theta 정규화
    theta = (theta + math.pi) % (2*math.pi) - math.pi

    # 경로기록
    path.append((x, y))

    screen.fill((25,25,30))

    if len(path) > 1:
        pts = [world_to_screen(px,py) for px,py in path]
        pygame.draw.lines(screen, (70,150,255), False, pts, 2)

    draw_robot(screen, x, y, theta)

    pygame.display.flip()
    clock.tick(int(1/dt))

pygame.quit()
