import random
import pygame
import sys
import math
from gicp import gicp

# Initialize Pygame
pygame.init()

# Constants
SCREEN_WIDTH = 1600
SCREEN_HEIGHT = 800
BG_COLOR = (255, 255, 255)
ROBOT_COLOR = (0, 128, 255)
OBSTACLE_COLOR = (255, 0, 0)
RAY_COLOR = (0, 255, 0)
INTERSECTION_COLOR = (0, 0, 255)
ROBOT_SIZE = 10
ROBOT_SPEED = 2
NUM_RAYS = 90
MAX_RAY_RANGE = 400
ROBOT_YAW_SPEED = 2
RIGHT_SIDE_WIDTH = SCREEN_WIDTH // 2
NOISE = 1
GICP_N_TICKS = 10

# Set up the display
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption('Robot Game with Raycasting')

# Robot starting position and yaw (angle)
robot_x = 10
robot_y = 10
robot_yaw = 0

# Define obstacles (rectangles and circles)
obstacles = [
    pygame.Rect(100, 100, 200, 50),  # Rectangles are defined by x, y, width, height
    pygame.Rect(400, 300, 50, 200),
    (600, 150, 50),  # Circles are defined by x, y, radius
    (200, 400, 75)
]

last_points = []

def cast_ray(robot_pos, angle):
    x1, y1 = robot_pos
    x2 = x1 + MAX_RAY_RANGE * math.cos(math.radians(angle))
    y2 = y1 + MAX_RAY_RANGE * math.sin(math.radians(angle))

    closest_intersection = None
    min_distance = float('inf')

    for obstacle in obstacles:
        if isinstance(obstacle, pygame.Rect):
            for line in [
                (obstacle.topleft, obstacle.topright),
                (obstacle.topright, obstacle.bottomright),
                (obstacle.bottomright, obstacle.bottomleft),
                (obstacle.bottomleft, obstacle.topleft),
            ]:
                intersection = ray_line_intersection((x1, y1), (x2, y2), line[0], line[1])
                if intersection:
                    dist = math.hypot(intersection[0] - x1, intersection[1] - y1)
                    if dist < min_distance:
                        min_distance = dist
                        closest_intersection = intersection
        elif isinstance(obstacle, tuple):
            intersection = ray_circle_intersection((x1, y1), (x2, y2), (obstacle[0], obstacle[1]), obstacle[2])
            if intersection:
                for point in intersection:
                    dist = math.hypot(point[0] - x1, point[1] - y1)
                    if dist < min_distance:
                        min_distance = dist
                        closest_intersection = point

    if closest_intersection:
        min_distance += random.uniform(-NOISE, NOISE)
        return min_distance
    else:
        return None

def ray_line_intersection(p1, p2, p3, p4):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0:
        return None

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
    if 0 <= t <= 1 and 0 <= u <= 1:
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
    return None

def ray_circle_intersection(p1, p2, center, radius):
    x1, y1 = p1
    x2, y2 = p2
    cx, cy = center

    dx, dy = x2 - x1, y2 - y1
    fx, fy = x1 - cx, y1 - cy

    a = dx * dx + dy * dy
    b = 2 * (fx * dx + fy * dy)
    c = (fx * fx + fy * fy) - radius * radius

    discriminant = b * b - 4 * a * c
    if discriminant >= 0:
        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)

        intersections = []
        if 0 <= t1 <= 1:
            intersections.append((x1 + t1 * dx, y1 + t1 * dy))
        if 0 <= t2 <= 1:
            intersections.append((x1 + t2 * dx, y1 + t2 * dy))

        return intersections if intersections else None
    return None

# Game loop
running = True
clock = pygame.time.Clock()

ticks = 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Handle keys
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        robot_yaw -= ROBOT_YAW_SPEED
    if keys[pygame.K_RIGHT]:
        robot_yaw += ROBOT_YAW_SPEED
    if keys[pygame.K_UP]:
        robot_x += ROBOT_SPEED * math.cos(math.radians(robot_yaw))
        robot_y += ROBOT_SPEED * math.sin(math.radians(robot_yaw))
    if keys[pygame.K_DOWN]:
        robot_x -= ROBOT_SPEED * math.cos(math.radians(robot_yaw))
        robot_y -= ROBOT_SPEED * math.sin(math.radians(robot_yaw))

    points = []
    for angle in range(robot_yaw, robot_yaw + 360, 360 // NUM_RAYS):
        distance = cast_ray((robot_x, robot_y), angle)
        if distance:
            rel_intersection = (
                distance * math.cos(math.radians(angle - robot_yaw)),
                distance * math.sin(math.radians(angle - robot_yaw))
            )
            abs_intersection = (
                robot_x + distance * math.cos(math.radians(angle)),
                robot_y + distance * math.sin(math.radians(angle))
            )
            points.append((rel_intersection, abs_intersection))

    if ticks % GICP_N_TICKS == 0:
        # GICP Stuff (def gicp(source_points, target_points, max_iterations=20, tolerance=1e-6, epsilon=1e-6):)
        # if len(last_points) > 0 and len(points) > 0:
        #     limit = min(len(last_points), len(points))
        #     last_points = last_points[:limit]
        #     points = points[:limit]
        #     transformation_matrix, all_transformations, source_cov_matrices, target_cov_matrices = gicp(last_points, points)

        last_points = points
    
    ticks += 1

    # Drawing
    screen.fill(BG_COLOR)

    # Left side drawing
    left_side = pygame.Surface((RIGHT_SIDE_WIDTH, SCREEN_HEIGHT))
    left_side.fill(BG_COLOR)

    # Draw obstacles
    for obstacle in obstacles:
        if isinstance(obstacle, pygame.Rect):
            pygame.draw.rect(left_side, OBSTACLE_COLOR, obstacle)
        elif isinstance(obstacle, tuple):
            pygame.draw.circle(left_side, OBSTACLE_COLOR, (obstacle[0], obstacle[1]), obstacle[2])

    # Raycasting and right side drawing
    right_side = pygame.Surface((RIGHT_SIDE_WIDTH, SCREEN_HEIGHT))
    right_side.fill(BG_COLOR)

    # Draw the circle on the right side
    circle_center = (RIGHT_SIDE_WIDTH // 2, SCREEN_HEIGHT // 2)
    pygame.draw.circle(right_side, (200, 200, 200), circle_center, MAX_RAY_RANGE, 1)
    pygame.draw.circle(right_side, (200, 200, 200), circle_center, ROBOT_SIZE)

    for _, abs_intersection in points:
        pygame.draw.line(left_side, RAY_COLOR, (robot_x + ROBOT_SIZE // 2, robot_y + ROBOT_SIZE // 2), abs_intersection)
        pygame.draw.circle(left_side, INTERSECTION_COLOR, (int(abs_intersection[0]), int(abs_intersection[1])), 3)

    for rel_intersection, _ in points:
        # Draw on the right side
        draw_x = RIGHT_SIDE_WIDTH // 2 + rel_intersection[0]
        draw_y = SCREEN_HEIGHT // 2 + rel_intersection[1]
        pygame.draw.circle(right_side, INTERSECTION_COLOR, (int(draw_x), int(draw_y)), 3)


    # Draw robot as circle
    pygame.draw.circle(left_side, ROBOT_COLOR, (int(robot_x + ROBOT_SIZE / 2), int(robot_y + ROBOT_SIZE / 2)), ROBOT_SIZE)

    # Blit both sides onto the main screen
    screen.blit(left_side, (0, 0))
    screen.blit(right_side, (RIGHT_SIDE_WIDTH, 0))
    pygame.draw.line(screen, (0, 0, 0), (robot_x + ROBOT_SIZE // 2, robot_y + ROBOT_SIZE // 2),
                     (robot_x + ROBOT_SIZE // 2 + ROBOT_SIZE * math.cos(math.radians(robot_yaw)),
                      robot_y + ROBOT_SIZE // 2 + ROBOT_SIZE * math.sin(math.radians(robot_yaw))), 2)

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
sys.exit()
