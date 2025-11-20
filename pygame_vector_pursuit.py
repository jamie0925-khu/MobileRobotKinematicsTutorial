import pygame
import numpy as np
import math
import random

#parameters
WIDTH, HEIGHT = 1000, 700
ROBOT_SIZE = 15
SPEED = 10.0
WHEELBASE = 20
LOOKAHEAD = 25.0
dt = 0.05

# vector_pursuit
def vector_pursuit(robot_pos, robot_yaw, path, lookahead):
    # 최근접 점 찾기
    dists = [np.linalg.norm(np.array(robot_pos)-np.array(p)) for p in path]
    nearest_index = np.argmin(dists)

    # Lookahead Point 선택
    target = path[-1]
    for i in range(nearest_index, len(path)):
        if np.linalg.norm(np.array(robot_pos)-np.array(path[i])) > lookahead:
            target = path[i]
            break

    # 로봇 방향 벡터
    heading_vec = np.array([math.cos(robot_yaw), math.sin(robot_yaw)])
    # 목표점 벡터
    target_vec = np.array([target[0]-robot_pos[0], target[1]-robot_pos[1]])

    # 벡터 사이 각도
    angle = math.atan2(np.cross(heading_vec, target_vec),
                       np.dot(heading_vec, target_vec))

    # Vector Pursuit 조향각
    delta = math.atan2(2*WHEELBASE*math.sin(angle)/lookahead, 1)
    return delta, target

# 경로 생성
def generate_zigzag():
    path = []
    x, y = 100, 350
    while x < 900:
        x += 20
        y += random.randint(-50, 50)
        y = max(100, min(600, y))
        path.append((x, y))
    return path

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

path = generate_zigzag()

robot_pos = [150, 150]
robot_yaw = 0.0
trajectory = []

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    delta, target = vector_pursuit(robot_pos, robot_yaw, path, LOOKAHEAD)

    # 로봇 업데이트
    robot_yaw += (SPEED/WHEELBASE)*math.tan(delta)*dt
    robot_pos[0] += SPEED*math.cos(robot_yaw)*dt
    robot_pos[1] += SPEED*math.sin(robot_yaw)*dt
    trajectory.append((robot_pos[0], robot_pos[1]))

    # 화면 그리기
    screen.fill((255,255,255))

    # 그리드
    grid_size = 50
    for x in range(0, WIDTH, grid_size):
        pygame.draw.line(screen, (200,200,200), (x,0), (x,HEIGHT), 1)
    for y in range(0, HEIGHT, grid_size):
        pygame.draw.line(screen, (200,200,200), (0,y), (WIDTH,y), 1)

    # 경로
    pygame.draw.lines(screen, (0,0,0), False, path, 3)

    # 궤적
    if len(trajectory) > 1:
        pygame.draw.lines(screen, (255,0,0), False, trajectory, 2)

    # 로봇
    cx, cy = int(robot_pos[0]), int(robot_pos[1])
    pygame.draw.circle(screen, (255,255,0), (cx,cy), ROBOT_SIZE)

    # 조향 화살표
    arrow_len = 40
    hx = cx + int(arrow_len*math.cos(robot_yaw+delta))
    hy = cy + int(arrow_len*math.sin(robot_yaw+delta))
    pygame.draw.line(screen, (0,200,0), (cx,cy), (hx,hy), 3)
    pygame.draw.circle(screen, (0,200,0), (hx,hy), 5)

    # 목표점
    pygame.draw.circle(screen, (0,255,255), (int(target[0]), int(target[1])), 5)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
