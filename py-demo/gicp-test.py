import numpy as np
from scipy.spatial import KDTree
import random
import pygame
from pygame.locals import QUIT
from scipy.optimize import fmin_cg

def compute_covariance_matrix(points, neighbors):
    cov_matrices = []
    for i in range(len(points)):
        cov_matrix = np.cov(neighbors[i].T)
        cov_matrices.append(cov_matrix)
    return np.array(cov_matrices)

def nearest_neighbors(source_points, target_points, k=6):
    tree = KDTree(target_points)
    distances, indices = tree.query(source_points, k=k)
    neighbors = target_points[indices]
    return neighbors

def rot_mat(theta):
    """Return a 2D rotation matrix given the angle theta."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])

def loss(x, a, b, M):
    """Compute the loss function for the given transformation parameters."""
    t = x[:2]
    R = rot_mat(x[2])
    residual = b - a @ R.T - t
    tmp = np.sum(M * residual[:, None, :], axis=2)
    return np.sum(residual * tmp)

def grad_loss(x, a, b, M):
    """Compute the gradient of the loss function."""
    t = x[:2]
    R = rot_mat(x[2])
    g = np.zeros(3)
    residual = b - a @ R.T - t
    tmp = np.sum(M * residual[:, None, :], axis=2)
    
    g[:2] = -2 * np.sum(tmp, axis=0)
    
    grad_R = -2 * (tmp.T @ a)
    grad_R_theta = np.array([[-np.sin(x[2]), -np.cos(x[2])], [np.cos(x[2]), -np.sin(x[2])]])
    g[2] = np.sum(grad_R * grad_R_theta)
    
    return g

def gicp(source_points, target_points, max_iterations=20, tolerance=1e-6, epsilon=1e-6):
    source_points = np.asarray(source_points)
    target_points = np.asarray(target_points)

    source_neighbors = nearest_neighbors(source_points, source_points)
    target_neighbors = nearest_neighbors(target_points, target_points)

    source_cov_matrices = compute_covariance_matrix(source_points, source_neighbors)
    target_cov_matrices = compute_covariance_matrix(target_points, target_neighbors)

    transformation_matrix = np.eye(3)
    all_transformations = [transformation_matrix]

    x = np.zeros(3)  # Parameters: [tx, ty, theta]

    last_min = np.inf
    for iteration in range(max_iterations):
        R = rot_mat(x[2])
        M = np.array([np.linalg.inv(target_cov_matrices[i] + R @ source_cov_matrices[i] @ R.T + epsilon * np.eye(2))
                      for i in range(len(source_points))])

        f = lambda x: loss(x, source_points, target_points, M)
        df = lambda x: grad_loss(x, source_points, target_points, M)

        out = fmin_cg(f=f, x0=x, fprime=df, disp=False, full_output=True)
        x = out[0]
        f_min = out[1]

        if np.abs(last_min - f_min) < tolerance:
            print("Converged at iteration", iteration)
            break
        last_min = f_min

        t = x[:2]
        theta = x[2]
        R = rot_mat(theta)
        transformation_matrix = np.eye(3)
        transformation_matrix[:2, :2] = R
        transformation_matrix[:2, 2] = t
        all_transformations.append(transformation_matrix)

    return transformation_matrix, all_transformations, source_cov_matrices, target_cov_matrices

def apply_transformation(cloud, T):
    return np.dot(cloud[:, :2], T[:2, :2].T) + T[:2, 2]

def generate_square_points(center, size, num_points_per_side):
    points = []
    half_size = size / 2
    sides = [np.linspace(center[0] - half_size, center[0] + half_size, num_points_per_side),
             np.linspace(center[1] - half_size, center[1] + half_size, num_points_per_side)]
    
    for x in sides[0]:
        points.append([x, center[1] - half_size])
    
    for y in sides[1]:
        points.append([center[0] + half_size, y])
    
    for x in sides[0]:
        points.append([x, center[1] + half_size])
    
    for y in sides[1]:
        points.append([center[0] - half_size, y])
    
    indices = random.sample(range(len(points)), len(points) // 2)
    points = [points[i] for i in range(len(points)) if i in indices]
    
    return np.array(points)

def generate_circle_points(center, radius, num_points):
    points = []
    for _ in range(num_points):
        angle = random.uniform(0, 2 * np.pi)
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        points.append([x, y])
    return np.array(points)

circle_center = (300, 150)
circle_radius = 100
num_points_circle = 30

square_center = (600, 250)
square_size = 200
num_points_square_side = 10

source_points_circle = generate_circle_points(circle_center, circle_radius, num_points_circle)
source_points_square = generate_square_points(square_center, square_size, num_points_square_side)

source_points = np.concatenate([source_points_circle, source_points_square])

def transform_points(points, translation, angle):
    rot_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    return np.dot(points, rot_matrix.T) + translation

translation = np.array([150, -50])
rotation_angle = np.pi / 6
target_points = transform_points(source_points, translation, rotation_angle)
target_points += np.random.normal(0, 5, target_points.shape)

def draw_points(points, color):
    for point in points:
        pygame.draw.circle(screen, color, point.astype(int), 3)

def draw_ellipse(surface, color, center, cov_matrix, n_std=2.0):
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

    # Create the ellipse surface
    ell = pygame.Surface((width, height), pygame.SRCALPHA)
    pygame.draw.ellipse(ell, color, (0, 0, width, height), 2)
    
    # Rotate the ellipse surface
    ell = pygame.transform.rotate(ell, -theta_degrees)
    
    # Calculate the new position to center the rotated ellipse
    rotated_rect = ell.get_rect(center=center)
    surface.blit(ell, rotated_rect)

pygame.init()
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("GICP Algorithm Visualization")

black = (0, 0, 0)
white = (255, 255, 255)
red = (255, 0, 0)
blue = (0, 0, 255)
green = (0, 255, 0)
yellow = (255, 255, 0)

running = True
clock = pygame.time.Clock()

T, all_transformations, source_cov_matrices, target_cov_matrices = gicp(source_points, target_points)

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

    draw_points(source_points, white)
    draw_points(target_points, blue)
    draw_points(transformed_source, red)
    
    for i, point in enumerate(source_points):
        draw_ellipse(screen, yellow, point, source_cov_matrices[i])
    
    for i, point in enumerate(target_points):
        draw_ellipse(screen, green, point, target_cov_matrices[i])

    pygame.display.flip()
    clock.tick(1)

pygame.quit()