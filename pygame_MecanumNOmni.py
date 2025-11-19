import pygame
import math
import sys

mode = "omni3"   # "mecanum" 또는 "omni3"

# 로봇 크기
ROBOT_W = 160
ROBOT_H = 120
R = 70   # omni3 삼각형 반경
WIDTH, HEIGHT = 1000, 700
FPS = 60
SPEED = 2.0
TURN_SPEED = 2.0

# 초기 pose
x, y = WIDTH // 2, HEIGHT // 2
theta = 0.0

# 메카넘 휠
def mecanum_wheels(vx, vy, omega):
    L = ROBOT_H / 2
    W = ROBOT_W / 2
    k = L + W

    fl = vx - vy - k * omega
    fr = vx + vy + k * omega
    rl = vx + vy - k * omega
    rr = vx - vy + k * omega
    return [fl, fr, rl, rr]

# 옴니 휠
def omni3_wheels(vx, vy, omega):
    wheel_dirs = [0, 2 * math.pi / 3, 4 * math.pi / 3]
    speeds = []

    for ang in wheel_dirs:
        v = vx * math.cos(ang) + vy * math.sin(ang) + R * omega
        speeds.append(v)

    return speeds


def draw_mecanum(screen, x, y, th, wheel_speeds):
    cos_t = math.cos(th)
    sin_t = math.sin(th)

    w = ROBOT_W / 2
    h = ROBOT_H / 2

    pts = [
        (x + cos_t * -w - sin_t * -h, y + sin_t * -w + cos_t * -h),
        (x + cos_t * w - sin_t * -h, y + sin_t * w + cos_t * -h),
        (x + cos_t * w - sin_t * h, y + sin_t * w + cos_t * h),
        (x + cos_t * -w - sin_t * h, y + sin_t * -w + cos_t * h),
    ]
    pygame.draw.polygon(screen, (150,150,220), pts, 3)

    # 정면 확인
    nx = x + cos_t * h
    ny = y + sin_t * h
    pygame.draw.line(screen, (255,0,0), (x,y), (nx,ny), 4)

    # 바퀴 (4개)
    offsets = [(-w, -h), (w, -h), (w, h), (-w, h)]
    for i, (ox, oy) in enumerate(offsets):
        wx = x + cos_t * ox - sin_t * oy
        wy = y + sin_t * ox + cos_t * oy

        pygame.draw.circle(screen, (255,255,255), (int(wx), int(wy)), 10)

        sp = wheel_speeds[i]
        dx = cos_t * sp * 0.05
        dy = sin_t * sp * 0.05
        pygame.draw.line(screen, (50,200,255), (wx,wy), (wx+dx,wy+dy), 3)

# 옴니 휠 
def draw_omni3(screen, x, y, th, wheel_speeds):
    # 삼각형 꼭짓점 (로봇 body)
    verts = []
    base_angles = [math.radians(90), math.radians(210), math.radians(330)]
    for ang in base_angles:
        vx = x + R * math.cos(th + ang)
        vy = y + R * math.sin(th + ang)
        verts.append((vx, vy))

    pygame.draw.polygon(screen, (150,220,150), verts, 3)

    # Omni 바퀴 3개
    wheel_dirs = [0, 2*math.pi/3, 4*math.pi/3]
    for i, (vx, vy) in enumerate(verts):
        pygame.draw.circle(screen,(255,255,255),(int(vx),int(vy)),10)

        ang = th + wheel_dirs[i]
        dx = math.cos(ang) * wheel_speeds[i] * 0.05
        dy = math.sin(ang) * wheel_speeds[i] * 0.05
        pygame.draw.line(screen,(50,255,255),(vx,vy),(vx+dx,vy+dy),3)

# 메인 루프
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

running = True
while running:
    dt = clock.tick(FPS) / 1000.0
    screen.fill((25,25,25))

    # 키보드 조작
    keys = pygame.key.get_pressed()
    vx = vy = omega = 0.0

    if keys[pygame.K_w]: vx = SPEED
    if keys[pygame.K_s]: vx = -SPEED
    if keys[pygame.K_a]: vy = SPEED
    if keys[pygame.K_d]: vy = -SPEED
    if keys[pygame.K_q]: omega = -TURN_SPEED
    if keys[pygame.K_e]: omega = TURN_SPEED

    if mode == "mecanum":
        wheels = mecanum_wheels(vx, vy, omega)
    else:
        wheels = omni3_wheels(vx, vy, omega)

    # 로봇 pose update
    x += (vx * math.cos(theta) - vy * math.sin(theta)) * 100 * dt
    y += (vx * math.sin(theta) + vy * math.cos(theta)) * 100 * dt
    theta += omega * dt

    if mode == "mecanum":
        draw_mecanum(screen, x, y, theta, wheels)
    else:
        draw_omni3(screen, x, y, theta, wheels)

    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False

    pygame.display.update()

pygame.quit()
sys.exit()
