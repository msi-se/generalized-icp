import random
import numpy as np
import pygame
from pygame.locals import QUIT
from scipy.spatial import cKDTree

# point to plane ICP Algorithm for 2D points with normals
def ptp(source_cloud, target_cloud, source_normals, target_normals, max_iterations=50, tolerance=1e-5, normal_weight=0.1):
    T = np.eye(3)
    all_transformations = [T]

    for iteration in range(max_iterations):
        correspondences = find_correspondences(source_cloud, target_cloud, source_normals, target_normals, T, normal_weight)
        T_new = compute_transformation(correspondences)
        T = T_new @ T  # Correct update by matrix multiplication
        all_transformations.append(T)

        if np.linalg.norm(T_new - np.eye(3)) < tolerance:
            print("Converged at iteration", iteration)
            break

    return T, all_transformations

def find_correspondences(source_cloud, target_cloud, source_normals, target_normals, T, normal_weight):
    transformed_source = apply_transformation(source_cloud, T)
    tree = cKDTree(target_cloud[:, :2])
    correspondences = []

    for idx, p_s in enumerate(transformed_source):
        closest_index = tree.query(p_s[:2], k=1)[1]
        p_t = target_cloud[closest_index]
        if np.dot(source_normals[idx], target_normals[closest_index]) > np.cos(np.pi / 4):
            correspondences.append((p_s, p_t))

    return correspondences

def compute_transformation(correspondences):
    A = []
    b = []

    for (p_s, p_t) in correspondences:
        A_i, b_i = create_linear_system(p_s, p_t)
        A.append(A_i)
        b.append(b_i)

    A = np.vstack(A)
    b = np.hstack(b)

    x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return create_transformation_matrix(x)

def create_linear_system(p_s, p_t):
    p_s = p_s[:2]
    p_t = p_t[:2]

    A_i = np.zeros((2, 3))
    A_i[0, 0] = 1
    A_i[1, 1] = 1
    A_i[0, 2] = -p_s[1]
    A_i[1, 2] = p_s[0]

    b_i = p_t - p_s

    return A_i, b_i

def create_transformation_matrix(x):
    T = np.eye(3)
    T[0, 2] = x[0]
    T[1, 2] = x[1]
    angle = x[2]
    T[0, 0] = np.cos(angle)
    T[0, 1] = -np.sin(angle)
    T[1, 0] = np.sin(angle)
    T[1, 1] = np.cos(angle)
    return T

def apply_transformation(cloud, T):
    # Apply transformation T to points
    return np.dot(cloud[:, :2], T[:2, :2].T) + T[:2, 2]

# Function to generate square points and normals
def generate_square_points(center, size, num_points_per_side):
    points = []
    normals = []
    half_size = size / 2
    sides = [np.linspace(center[0] - half_size, center[0] + half_size, num_points_per_side),
             np.linspace(center[1] - half_size, center[1] + half_size, num_points_per_side)]
    
    # Bottom side
    for x in sides[0]:
        points.append([x, center[1] - half_size])
        normals.append([0, -1])
    
    # Right side
    for y in sides[1]:
        points.append([center[0] + half_size, y])
        normals.append([1, 0])
    
    # Top side
    for x in sides[0]:
        points.append([x, center[1] + half_size])
        normals.append([0, 1])
    
    # Left side
    for y in sides[1]:
        points.append([center[0] - half_size, y])
        normals.append([-1, 0])
    
    # random remove half of the points
    indices = random.sample(range(len(points)), len(points) // 2)
    points = [points[i] for i in range(len(points)) if i in indices]
    normals = [normals[i] for i in range(len(normals)) if i in indices]
    
    return np.array(points), np.array(normals)

# Function to generate points and normals from a circle
def generate_circle_points(center, radius, num_points):
    points = []
    normals = []
    for _ in range(num_points):
        angle = random.uniform(0, 2 * np.pi)
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        points.append([x, y])
        normals.append([np.cos(angle), np.sin(angle)])
    return np.array(points), np.array(normals)

# Generate source and target clouds
circle_center = (300, 150)
circle_radius = 100
num_points_circle = 30

square_center = (600, 250)
square_size = 200
num_points_square_side = 10

source_points_circle, source_normals_circle = generate_circle_points(circle_center, circle_radius, num_points_circle)
source_points_square, source_normals_square = generate_square_points(square_center, square_size, num_points_square_side)

# Combine shapes into one cloud
source_points = np.vstack((source_points_circle, source_points_square))
source_normals = np.vstack((source_normals_circle, source_normals_square))

def transform_points(points, translation, angle):
    rot_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    return np.dot(points, rot_matrix.T) + translation

# Apply transformation to create target cloud
translation = np.array([150, -50])
rotation_angle = np.pi / 6  # 30 degrees
target_points = transform_points(source_points, translation, rotation_angle)
target_normals = transform_points(source_normals, np.zeros(2), rotation_angle)  # Normals rotate only

# Pygame visualization
def draw_points_normals(points, normals, color):
    for point, normal in zip(points, normals):
        pygame.draw.circle(screen, color, point.astype(int), 3)
        end_point = point + 20 * normal
        pygame.draw.line(screen, color, point.astype(int), end_point.astype(int), 1)

pygame.init()
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("point to plane ICP Algorithm Visualization")

black = (0, 0, 0)
white = (255, 255, 255)
red = (255, 0, 0)
blue = (0, 0, 255)

running = True
clock = pygame.time.Clock()

T, all_transformations = ptp(source_points, target_points, source_normals, target_normals)

step = 0
max_steps = len(all_transformations)
while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False

    screen.fill(black)
    
    if step < max_steps:
        current_T = all_transformations[step]
        transformed_source = apply_transformation(source_points, current_T)
        step += 1
        
    transformed_source_normals = np.dot(source_normals, current_T[:2, :2].T)

    draw_points_normals(source_points, source_normals, white)
    draw_points_normals(target_points, target_normals, blue)
    draw_points_normals(transformed_source, transformed_source_normals, red)
    
    pygame.display.flip()
    clock.tick(1)  # Control the speed of the visualization

pygame.quit()