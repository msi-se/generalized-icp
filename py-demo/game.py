import random
import pygame
import sys
import math
from gicp import gicp, apply_transformation
import numpy as np

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
START_X = 20
START_Y = 20
ROBOT_SPEED = 2
NUM_RAYS = 180
MAX_RAY_RANGE = 400
ROBOT_YAW_SPEED = 2
RIGHT_SIDE_WIDTH = SCREEN_WIDTH // 2
NOISE = 2
GICP_N_TICKS = 10

# Set up the display
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption('Robot Game with Raycasting')

# Robot starting position and yaw (angle)
robot_x = START_X
robot_y = START_Y
robot_yaw = 0

# Define obstacles (rectangles and circles)
obstacles = [
    pygame.Rect(100, 100, 200, 50),  # Rectangles are defined by x, y, width, height
    pygame.Rect(400, 300, 50, 200),
    (600, 150, 50),  # Circles are defined by x, y, radius
    (200, 400, 75)
]

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

def draw_ellipse(surface, color, center, cov_matrix, n_std=2.0, alpha=60):
    vals, vecs = np.linalg.eigh(cov_matrix)
    vals = np.maximum(vals, 1e-6) 
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    
    # Angle of rotation (theta)
    largest_eigenvec = vecs[:, 0]
    theta = np.arctan2(largest_eigenvec[1], largest_eigenvec[0])
    theta_degrees = np.degrees(theta)

    # Calculate width and height of the ellipse
    width, height = 2 * n_std * np.sqrt(vals)

    if np.isnan(width) or np.isnan(height):
        return
    
    # Create the ellipse surface
    ell = pygame.Surface((width, height), pygame.SRCALPHA)
    ell.set_alpha(alpha)
    pygame.draw.ellipse(ell, color, (0, 0, width, height), 2)
    
    # Rotate the ellipse surface
    ell = pygame.transform.rotate(ell, -theta_degrees)
    
    # Calculate the new position to center the rotated ellipse
    rotated_rect = ell.get_rect(center=center)
    surface.blit(ell, rotated_rect)

# Game loop
running = True
clock = pygame.time.Clock()

ticks = 0
source_points = []
target_points = []
source_cov_matrices = []
target_cov_matrices = []
transformation_matrix = []
transformation_matrixes = []

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
        source_points = target_points
        target_points = points
        if len(source_points) > 0 and len(target_points) > 0:
            _source_points = [rel_intersection for rel_intersection, _ in source_points]
            _target_points = [rel_intersection for rel_intersection, _ in target_points]
            transformation_matrix, all_transformations, source_cov_matrices, target_cov_matrices = gicp(
                _source_points,
                _target_points,
                max_distance_nearest_neighbors=200,
            )
            transformation_matrixes.append(transformation_matrix)
            print(len(source_cov_matrices))
    
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
        pygame.draw.line(left_side, RAY_COLOR, (robot_x, robot_y), abs_intersection)
        pygame.draw.circle(left_side, INTERSECTION_COLOR, (int(abs_intersection[0]), int(abs_intersection[1])), 3)

    for i, (rel_intersection, _) in enumerate(target_points):
        # Draw on the right side
        draw_x = RIGHT_SIDE_WIDTH // 2 + rel_intersection[0]
        draw_y = SCREEN_HEIGHT // 2 + rel_intersection[1]
        pygame.draw.circle(right_side, (0, 0, 255), (int(draw_x), int(draw_y)), 3)
        if len(target_cov_matrices) > i:
            draw_ellipse(right_side, (128, 128, 255), (int(draw_x), int(draw_y)), target_cov_matrices[i])
    
    # apple transformation_matrixs to source points and draw them
    if len(transformation_matrix) > 0:
        for i, (rel_intersection, _) in enumerate(source_points):
            draw_x = RIGHT_SIDE_WIDTH // 2 + rel_intersection[0]
            draw_y = SCREEN_HEIGHT // 2 + rel_intersection[1]
            pygame.draw.circle(right_side, (192, 192, 192), (int(draw_x), int(draw_y)), 3)
            if len(source_cov_matrices) > i:
                draw_ellipse(right_side, (192, 192, 192), (int(draw_x), int(draw_y)), source_cov_matrices[i])

        # Draw transformed source points
        rel_intersections = np.asarray([[rel_intersection[0], rel_intersection[1]] for rel_intersection, _ in source_points])
        transformed_points = apply_transformation(rel_intersections, transformation_matrix)
        for i, transformed_point in enumerate(transformed_points):
            draw_x = RIGHT_SIDE_WIDTH // 2 + transformed_point[0]
            draw_y = SCREEN_HEIGHT // 2 + transformed_point[1]
            pygame.draw.circle(right_side, (255, 0, 0), (int(draw_x), int(draw_y)), 3)


    # Draw robot as circle
    pygame.draw.circle(left_side, ROBOT_COLOR, (int(robot_x), int(robot_y)), ROBOT_SIZE)
    pygame.draw.line(left_side, (0, 0, 0), (robot_x, robot_y),
                     (robot_x + ROBOT_SIZE * math.cos(math.radians(robot_yaw)),
                      robot_y + ROBOT_SIZE * math.sin(math.radians(robot_yaw))), 2)
    
    def transformation_matrix_to_vector(matrix):
        return [matrix[0][2], matrix[1][2]]
    
    # Blit both sides onto the main screen
    screen.blit(left_side, (0, 0))
    screen.blit(right_side, (RIGHT_SIDE_WIDTH, 0))

    pygame.display.flip()
    clock.tick(15)

pygame.quit()
sys.exit()
