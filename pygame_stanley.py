import pygame
import numpy as np
import math

# Parameters
dt = 0.1
k = 2
LENGTH, WIDTH = 4.715, 1.910
L = 2.875
BACKTOWHEEL = 1.0
WHEEL_LEN, WHEEL_WIDTH = 0.3, 0.2
TREAD = 0.8
max_steering = np.radians(30)

WIDTH, HEIGHT = 1000, 700
SCALE = 10.0
CENTER = (WIDTH//2, HEIGHT//2)


# 따라갈 경로 생성
def make_ref(road="linear"):
    if road == "linear":
        ref_xs = np.linspace(0, 500, 500)
        ref_ys = np.zeros_like(ref_xs) + 1.5
        ref_yaws = np.arctan(np.gradient(ref_ys, ref_xs))
    elif road == "sin":
        ref_xs = np.linspace(0, 500, 500)
        ref_ys = 5 * np.sin(0.2*ref_xs) + 1
        ref_yaws = np.arctan(np.gradient(ref_ys, ref_xs))
    elif road == "circle":
        theta = np.linspace(0, 2*np.pi, 500)
        ref_xs = 20 * np.cos(theta) - 20
        ref_ys = 20 * np.sin(theta)
        ref_yaws = np.pi/2 + theta
    return ref_xs, ref_ys, ref_yaws

# 차량 모델 
class VehicleModel:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw*np.pi/180
        self.v = v

    def update(self, steer):
        self.yaw += self.v / L * np.tan(steer) * dt
        self.yaw = self.yaw % (2.0*np.pi)
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

# stanley
def stanley_control(x, y, yaw, v, ref_xs, ref_ys, ref_yaws):
    front_x = x + L * np.cos(yaw)
    front_y = y + L * np.sin(yaw)

    dists = np.hypot(front_x - ref_xs, front_y - ref_ys)
    min_index = int(np.argmin(dists))

    ref_x = ref_xs[min_index]
    ref_y = ref_ys[min_index]
    ref_yaw = ref_yaws[min_index]

    dx = ref_x - front_x
    dy = ref_y - front_y
    perp_vec = [np.cos(ref_yaw + np.pi/2), np.sin(ref_yaw + np.pi/2)]
    cte = np.dot([dx, dy], perp_vec)

    psi = normalize_angle(ref_yaw - yaw)
    cte_term = math.atan2(k*cte, max(v, 1e-3))

    steer = psi + cte_term
    steer = np.clip(steer, -max_steering, max_steering)
    return steer, ref_x, ref_y


def world_to_screen(x, y):
    sx = CENTER[0] + int(x*SCALE)
    sy = CENTER[1] - int(y*SCALE)
    return sx, sy

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# 경로 생성
ref_xs, ref_ys, ref_yaws = make_ref(road="circle")

# 차량 초기화
model = VehicleModel(x=10.0, y=15.0, yaw=0, v=5.0) # 시나리오 1
#model = VehicleModel(x=0.0, y=0.0, yaw=270,v=5.0) # 시나리오 2

trajectory = []

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Stanley 제어
    steer, rx, ry = stanley_control(model.x, model.y, model.yaw, model.v,
                                    ref_xs, ref_ys, ref_yaws)

    # 차량 업데이트
    model.update(steer)
    trajectory.append((model.x, model.y))

    screen.fill((255,255,255))

    # 경로
    pts = [world_to_screen(ref_xs[i], ref_ys[i]) for i in range(len(ref_xs))]
    pygame.draw.lines(screen, (0,0,0), False, pts, 2)

    # 궤적
    if len(trajectory) > 1:
        traj_pts = [world_to_screen(px, py) for (px,py) in trajectory]
        pygame.draw.lines(screen, (255,0,0), False, traj_pts, 2)

    # 차량 위치
    cx, cy = world_to_screen(model.x, model.y)
    pygame.draw.circle(screen, (255,255,0), (cx,cy), 10)

    # 조향 화살표
    arrow_len = 40
    hx = cx + int(arrow_len*math.cos(model.yaw+steer))
    hy = cy - int(arrow_len*math.sin(model.yaw+steer))
    pygame.draw.line(screen, (0,200,0), (cx,cy), (hx,hy), 3)
    pygame.draw.circle(screen, (0,200,0), (hx,hy), 5)

    # 참조점
    rx_s, ry_s = world_to_screen(rx, ry)
    pygame.draw.circle(screen, (0,255,255), (rx_s, ry_s), 5)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
