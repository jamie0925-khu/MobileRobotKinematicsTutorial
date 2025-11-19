import pygame
import numpy as np
import math
import random

# parameters
WIDTH, HEIGHT = 1000, 700
ROBOT_SIZE = 15
LOOKAHEAD = 40.0  # lookahead distance # 이것을 바꾸면서 시뮬레이션 진행
SPEED = 2.0        
WHEELBASE = 20     

def pure_pursuit(robot_pos, robot_yaw, path, lookahead):
    dists = [np.linalg.norm(np.array(robot_pos) - np.array(p)) for p in path]
    nearest_index = np.argmin(dists)

    Ld = lookahead
    target = path[-1]
    for i in range(nearest_index, len(path)):
        if np.linalg.norm(np.array(robot_pos) - np.array(path[i])) > Ld:
            target = path[i]
            break

    dx = target[0] - robot_pos[0]
    dy = target[1] - robot_pos[1]
    angle_to_target = math.atan2(dy, dx)
    alpha = angle_to_target - robot_yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))

    delta = math.atan2(2 * WHEELBASE * math.sin(alpha) / Ld, 1)

    return delta, target

# 경로 만들기 
def generate_rectangle():
    path = []
    for x in range(100, 900): path.append((x, 100))
    for y in range(100, 500): path.append((900, y))
    for x in range(900, 100, -1): path.append((x, 500))
    for y in range(500, 100, -1): path.append((100, y))
    return path

def generate_8_shape():
    t = np.linspace(0, 2*np.pi, 800)
    path = []
    for i in t:
        x = 500 + 300 * math.sin(i)
        y = 350 + 150 * math.sin(i) * math.cos(i)
        path.append((x, y))
    return path

def generate_zigzag():
    path = []
    x = 100
    y = 350
    while x < 900:
        x += 20
        y += random.randint(-50, 50)
        y = max(100, min(600, y))
        path.append((x, y))
    return path

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# 경로 생성 
# path = generate_rectangle()     
# path = generate_8_shape()
path = generate_zigzag()

robot_pos = [150, 150]
robot_yaw = 0.0
trajectory = []   # 로봇이 지나간 궤적 저장

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    delta, target = pure_pursuit(robot_pos, robot_yaw, path, LOOKAHEAD)

    # 로봇 움직임 업데이트
    robot_yaw += math.tan(delta) * SPEED / WHEELBASE
    robot_pos[0] += SPEED * math.cos(robot_yaw)
    robot_pos[1] += SPEED * math.sin(robot_yaw)

    trajectory.append((robot_pos[0], robot_pos[1]))

    screen.fill((255, 255, 255)) 

    # 맵생성
    grid_size = 50
    for x in range(0, WIDTH, grid_size):
        pygame.draw.line(screen, (60, 60, 60), (x, 0), (x, HEIGHT), 1)
    for y in range(0, HEIGHT, grid_size):
        pygame.draw.line(screen, (60, 60, 60), (0, y), (WIDTH, y), 1)

    # 경로 두께 조절
    pygame.draw.lines(screen, (0, 0, 0), False, path, 3)


    # 로봇 경로 표시
    if len(trajectory) > 1:
        pygame.draw.lines(screen, (255, 0, 0), False, trajectory, 2)

    pygame.draw.circle(screen, (0, 255, 255), (int(target[0]), int(target[1])), 5)
    pygame.draw.circle(screen, (255, 255, 0), (int(robot_pos[0]), int(robot_pos[1])), ROBOT_SIZE)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
