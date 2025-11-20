import pygame
import numpy as np
import math
import matplotlib.pyplot as plt

#parameters
obstacle_count = 2
OBSTACLE_DIST = 350
obstacle_radius = 30
AMPLITUDE = 150

WIDTH, HEIGHT = 900, 600
ROBOT_SIZE = 15
MIN_LOOKAHEAD = 10.0
WHEELBASE = 20
GAIN_X = 1.5

GAIN_OBST = 1.0
PROX_DIST = 80
MIN_SPEED = 1.0
T = 4.0  # 최대 선속도
R_MIN = 60

# 장애물 배치
obstacle_centers = [(250 + OBSTACLE_DIST * i, 300) for i in range(obstacle_count)]

# 경로 생성 
x_end = 250 + OBSTACLE_DIST * (obstacle_count - 1) + 350  
x_path = np.linspace(120, x_end, 900)
x_range = obstacle_centers[-1][0] - obstacle_centers[0][0]
y_path = 300 + AMPLITUDE * np.sin((x_path - 120) / x_range * 2 * np.pi)
path = list(zip(x_path, y_path))

def dynamic_lookahead(speed):
    return max(MIN_LOOKAHEAD, speed * GAIN_X)

def linear_Interpolation(center, radius, p0, p1):
    center = np.array(center, dtype=float)
    p0 = np.array(p0, dtype=float)
    p1 = np.array(p1, dtype=float)
    d = p1 - p0
    a = np.dot(d, d)
    if a == 0.0:
        return None
    f = p0 - center
    b = 2.0 * np.dot(d, f)
    c = np.dot(f, f) - radius * radius
    disc = b * b - 4 * a * c
    if disc < 0:
        return None
    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / (2 * a)
    t2 = (-b + sqrt_disc) / (2 * a)
    candidates = []
    for t in (t1, t2):
        if 0.0 <= t <= 1.0:
            candidates.append(p0 + t * d)
    if not candidates:
        return None
    dists = [np.linalg.norm(pt - center) for pt in candidates]
    return candidates[int(np.argmin(dists))]

def regulated_pure_pursuit(robot_pos, robot_yaw, path, speed):
    Ld = dynamic_lookahead(speed)
    dists = [np.linalg.norm(np.array(robot_pos) - np.array(p)) for p in path]
    nearest_index = int(np.argmin(dists))
    target = np.array(path[-1], dtype=float)
    for i in range(max(1, nearest_index), len(path)):
        dist_i = np.linalg.norm(np.array(robot_pos) - np.array(path[i]))
        if dist_i > Ld:
            p1 = np.array(path[i], dtype=float)
            p0 = np.array(path[i - 1], dtype=float)
            intersection = linear_Interpolation(robot_pos, Ld, p0, p1)
            if intersection is not None:
                target = intersection
            else:
                d = p1 - p0
                n = np.linalg.norm(d)
                if n > 1e-6:
                    dir_vec = d / n
                    to_p1 = p1 - np.array(robot_pos, dtype=float)
                    m = np.linalg.norm(to_p1)
                    if m > 1e-6:
                        dir2 = to_p1 / m
                        target = np.array(robot_pos, dtype=float) + dir2 * Ld
                    else:
                        target = p1
                else:
                    target = p1
            break
    dx = target[0] - robot_pos[0]
    dy = target[1] - robot_pos[1]
    angle_to_target = math.atan2(dy, dx)
    alpha = angle_to_target - robot_yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    delta = math.atan2(2.0 * WHEELBASE * math.sin(alpha) / Ld, 1.0)
    return delta, (float(target[0]), float(target[1]))

def regulated_speed(v_max, delta, robot_pos, obstacles, obs_radius,
                    wheelbase=WHEELBASE, r_min=R_MIN, tmax=T, prox_dist=PROX_DIST, v_min=MIN_SPEED):
    # 곡률(회전반경) 기반 속도제한
    if abs(math.tan(delta)) < 1e-5:
        radius = 1e6
    else:
        radius = abs(wheelbase / math.tan(delta))
    vt_curve = min(tmax, abs(v_max * radius / tmax)) # max속도 or regulated
    # 장애물 proximity 감속
    min_dist = float('inf')
    for center in obstacles:
        dist = np.linalg.norm(np.array(robot_pos) - np.array(center)) - obs_radius - ROBOT_SIZE
        if dist < min_dist:
            min_dist = dist
    dist_to_obst = max(min_dist, 0.0)
    if dist_to_obst < prox_dist:
        factor = (dist_to_obst / prox_dist) * GAIN_OBST
        vt_curve = max(v_min, vt_curve * factor)
    else:
        vt_curve = max(v_min, vt_curve)
    return vt_curve

# pygame 생성
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

robot_pos = [float(x_path[0]), float(y_path[0])]
robot_yaw = 0.0
trajectory = []
speeds = []

MAX_SPEED = T

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # 항상 최대 속도로 곡률/장애물 감속 동시 적용
    delta, target = regulated_pure_pursuit(robot_pos, robot_yaw, path, MAX_SPEED)
    current_speed = regulated_speed(MAX_SPEED, delta, robot_pos, obstacle_centers, obstacle_radius)
    speeds.append(current_speed)

    robot_yaw += math.tan(delta) * current_speed / WHEELBASE
    robot_pos[0] += current_speed * math.cos(robot_yaw)
    robot_pos[1] += current_speed * math.sin(robot_yaw)
    trajectory.append((robot_pos[0], robot_pos[1]))

    screen.fill((255, 255, 255))

    # 그리드
    grid_size = 50
    for x in range(0, WIDTH, grid_size):
        pygame.draw.line(screen, (220, 220, 220), (x, 0), (x, HEIGHT), 1)
    for y in range(0, HEIGHT, grid_size):
        pygame.draw.line(screen, (220, 220, 220), (0, y), (WIDTH, y), 1)

    # 경로
    pygame.draw.lines(screen, (0, 60, 180), False, path, 3)
    if len(trajectory) > 1:
        pygame.draw.lines(screen, (255, 0, 0), False, trajectory, 2)

    # 장애물(원)
    for center in obstacle_centers:
        pygame.draw.circle(screen, (80, 80, 255), (int(center[0]), int(center[1])), obstacle_radius, 2)

    # 목표점/로봇
    pygame.draw.circle(screen, (0, 255, 255), (int(target[0]), int(target[1])), 6)
    pygame.draw.circle(screen, (255, 200, 0), (int(robot_pos[0]), int(robot_pos[1])), ROBOT_SIZE)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()

plt.figure(figsize=(10,4))
plt.plot(speeds)
plt.xlabel('Time step')
plt.ylabel('Robot speed')
plt.title('Regulated Robot Speed Over Time')
plt.grid(True)
plt.show()
