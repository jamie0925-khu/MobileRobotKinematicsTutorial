import pygame
import numpy as np
import math
import random

# parameters
WIDTH, HEIGHT = 1000, 700
ROBOT_SIZE = 15
MIN_LOOKAHEAD = 10.0
WHEELBASE = 20
GAIN_X = 1.5 # 4.0

# 최소값 보장
def dynamic_lookahead(speed):
    return max(MIN_LOOKAHEAD, speed * GAIN_X)

# 조향각에 따른 속도 변화
def update_velocity(delta):
    max_speed = 4.0
    min_speed = 1.0
    factor = max(0.0, 1.0 - abs(delta)) 
    speed = min_speed + (max_speed - min_speed) * factor  # delta가 커질수록 속도를 줄임 
    return speed

def linear_Interpolation(center, radius, p0, p1):
    """
    원(center, radius)과 선분 p0->p1의 교점을 계산.
    선분 파라미터식: L(t) = p0 + t*(p1-p0), t in [0,1]
    반환: 교점 중 로봇 중심에 가장 가까운 t(>=0)로 계산한 점. 없으면 None.
    """
    center = np.array(center, dtype=float)
    p0 = np.array(p0, dtype=float)
    p1 = np.array(p1, dtype=float)
    d = p1 - p0
    a = np.dot(d, d)
    if a == 0.0:
        return None  # p0==p1 (0-길이 세그먼트)

    f = p0 - center
    # |p0 + t*d - center|^2 = radius^2
    # (d·d) t^2 + 2(d·f) t + (f·f - r^2) = 0
    b = 2.0 * np.dot(d, f)
    c = np.dot(f, f) - radius * radius
    disc = b*b - 4*a*c
    if disc < 0:
        return None  # 교점 없음

    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / (2*a)
    t2 = (-b + sqrt_disc) / (2*a)

    candidates = []
    for t in (t1, t2):
        if 0.0 <= t <= 1.0:
            candidates.append(p0 + t*d)

    if not candidates:
        return None
    # 원 중심에 더 가까운 교점 선택
    dists = [np.linalg.norm(pt - center) for pt in candidates]
    return candidates[int(np.argmin(dists))]

def pure_pursuit_revisited(robot_pos, robot_yaw, path, speed):
    Ld = dynamic_lookahead(speed)

    # 가장 가까운 waypoint 인덱스
    dists = [np.linalg.norm(np.array(robot_pos) - np.array(p)) for p in path]
    nearest_index = int(np.argmin(dists))

    target = np.array(path[-1], dtype=float)

    # 검색 원 밖의 첫 waypoint를 찾고, 해당 구간(p_{i-1}~p_i)과 원의 교점을 실제 목표로 설정
    for i in range(max(1, nearest_index), len(path)):
        dist_i = np.linalg.norm(np.array(robot_pos) - np.array(path[i]))
        if dist_i > Ld:
            p1 = np.array(path[i], dtype=float)
            p0 = np.array(path[i - 1], dtype=float)

            # circle-line intersection: 원 중심=로봇 위치, 반지름=Ld, 선분=p0->p1
            intersection = linear_Interpolation(robot_pos, Ld, p0, p1)

            if intersection is not None:
                target = intersection
            else:
                # 교점이 없으면 보간 실패 -> 세그먼트 방향으로 근사치
                d = p1 - p0
                n = np.linalg.norm(d)
                if n > 1e-6:
                    dir_vec = d / n
                    #로봇 중심에서 세그먼트 방향으로 Ld 이동
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

    # 조향각 계산 (Pure Pursuit)
    dx = target[0] - robot_pos[0]
    dy = target[1] - robot_pos[1]
    angle_to_target = math.atan2(dy, dx)
    alpha = angle_to_target - robot_yaw

    # [-pi, pi]로 정규화
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))

    delta = math.atan2(2.0 * WHEELBASE * math.sin(alpha) / Ld, 1.0)
    return delta, (float(target[0]), float(target[1]))

# 경로 생성
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
        x += 30   
        y += random.randint(-50, 50)
        y = max(100, min(600, y))
        path.append((x, y))
    return path

# pygame 생성
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# 경로 생성
# path = generate_rectangle()
# path = generate_8_shape()
path = generate_zigzag()

robot_pos = [150.0, 150.0]
robot_yaw = 0.0
trajectory = []   # 로봇이 지나간 궤적 저장

# 초기 속도
current_speed = 2.0

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # pure_pursuit_revisited
    delta, target = pure_pursuit_revisited(robot_pos, robot_yaw, path, current_speed)

    # 속도 갱신 (직선/곡선에 따라)
    current_speed = update_velocity(delta)

    # 로봇 움직임 업데이트
    robot_yaw += math.tan(delta) * current_speed / WHEELBASE
    robot_pos[0] += current_speed * math.cos(robot_yaw)
    robot_pos[1] += current_speed * math.sin(robot_yaw)

    trajectory.append((robot_pos[0], robot_pos[1]))

    screen.fill((255, 255, 255))

    # 맵생성
    grid_size = 50
    for x in range(0, WIDTH, grid_size):
        pygame.draw.line(screen, (60, 60, 60), (x, 0), (x, HEIGHT), 1)
    for y in range(0, HEIGHT, grid_size):
        pygame.draw.line(screen, (60, 60, 60), (0, y), (WIDTH, y), 1)

    # 경로
    pygame.draw.lines(screen, (0, 0, 0), False, path, 3)

    # 로봇 궤적
    if len(trajectory) > 1:
        pygame.draw.lines(screen, (255, 0, 0), False, trajectory, 2)

    # 목표점/로봇 표시
    pygame.draw.circle(screen, (0, 255, 255), (int(target[0]), int(target[1])), 5)
    pygame.draw.circle(screen, (255, 255, 0), (int(robot_pos[0]), int(robot_pos[1])), ROBOT_SIZE)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
