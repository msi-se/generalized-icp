import numpy as np
from scipy.spatial import KDTree
import random
import pygame
from pygame.locals import QUIT
from scipy.optimize import fmin_cg


def compute_covariance_matrix(points, neighbors):
    """Compute the covariance matrices for each point in the cloud."""
    num_points = len(points)
    cov_matrices = np.zeros((num_points, 2, 2))
    for i in range(num_points):
        cov_matrices[i] = np.cov(neighbors[i].T)
    return cov_matrices

def nearest_neighbors(source_points, target_points, k=6):
    """Find the k nearest neighbors of each point in the source cloud."""
    tree = KDTree(target_points)
    distances, indices = tree.query(source_points, k=k)
    neighbors = target_points[indices]
    return neighbors

def rot_mat(theta):
    """Return a 2D rotation matrix given the angle theta."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])

def loss(offset, source_points, target_points, weight_matrices):
    """Compute the loss function for the given transformation parameters."""
    translation = offset[:2]
    rotation_matrix = rot_mat(offset[2])
    residuals = target_points - source_points @ rotation_matrix.T - translation
    weighted_residuals = np.sum(weight_matrices * residuals[:, None, :], axis=2)
    return np.sum(residuals * weighted_residuals)

def grad_loss(offset, source_points, target_points, weight_matrices):
    """Compute the gradient of the loss function."""
    translation = offset[:2]
    rotation_matrix = rot_mat(offset[2])
    gradient = np.zeros(3)
    residuals = target_points - source_points @ rotation_matrix.T - translation
    weighted_residuals = np.sum(weight_matrices * residuals[:, None, :], axis=2)
    
    # Gradient with respect to translation
    gradient[:2] = -2 * np.sum(weighted_residuals, axis=0)
    
    # Gradient with respect to rotation
    grad_rotation = -2 * (weighted_residuals.T @ source_points)
    grad_rotation_theta = np.array([[-np.sin(offset[2]), -np.cos(offset[2])], [np.cos(offset[2]), -np.sin(offset[2])]])
    gradient[2] = np.sum(grad_rotation * grad_rotation_theta)
    
    return gradient


def offset_to_transformation_matrix(offset):
    """Convert the offset parameters (tx, ty, theta) to a transformation matrix."""
    translation = offset[:2]
    theta = offset[2]
    rotation_matrix = rot_mat(theta)
    transformation_matrix = np.eye(3)
    transformation_matrix[:2, :2] = rotation_matrix
    transformation_matrix[:2, 2] = translation
    return transformation_matrix

def gicp(source_points, target_points, max_iterations=60, tolerance=1e-6, epsilon=1e-6, max_distance_correspondence=150, max_distance_nearest_neighbors=10):
    """
    Generalized Iterative Closest Point (GICP) algorithm to align two 2D point clouds.
    :param source_points: Source point cloud (Nx2)
    :param target_points: Target point cloud (Nx2)
    :param max_iterations: Maximum number of iterations
    :param tolerance: Tolerance for convergence
    :param epsilon: Small value to avoid numerical issues
    :return: Transformation matrix, all transformations, source covariance matrices, target covariance matrices

    Algorithm from the Stanford paper
        Generalized-ICP (Segal, Haehnel, Thrun)

        Input: Two point clouds: A = {ai}, B = {bi}
        An initial transformation: T0

        Output: The correct transformation, T, which aligns A and B

        1: T ← T0
        2: while not converged do
        3:     for i ← 1 to N do
        4:         mi ← FindClosestPointInA(T · bi)
        5:         if ||mi - T · bi|| ≤ dmax then
        6:             wi ← 1
        7:         else
        8:             wi ← 0
        9:         end if
        10:    end for
        11:    T ← argmin_T Σi (di^T (C_Bi + T C_Ai T^T)^-1 di)
        12: end while

    """

    # Convert points to numpy arrays if they are not already
    source_points = np.asarray(source_points)
    target_points = np.asarray(target_points)

    # Compute nearest neighbors and covariance matrices
    target_neighbors = nearest_neighbors(target_points, target_points)
    target_cov_matrices = compute_covariance_matrix(target_points, target_neighbors)

    # Initialize transformation matrix and parameters
    transformation_matrix = np.eye(3)
    all_transformations = [transformation_matrix]
    offset = np.zeros(3)  # Parameters: [tx, ty, theta]
    last_min_loss = np.inf

    for iteration in range(max_iterations):

        rotation_matrix = np.eye(2)

        # Find the closest points in the target cloud
        transformed_source_points = apply_transformation(source_points, transformation_matrix)
        transformed_source_neighbors = nearest_neighbors(transformed_source_points, transformed_source_points)
        source_cov_matrices = compute_covariance_matrix(transformed_source_points, transformed_source_neighbors)

        corresponding_target_points = np.zeros_like(source_points)
        weight_matrices = np.zeros((len(source_points), 2, 2))
        tree = KDTree(target_points)
        distances = np.zeros(len(source_points))
        for i, source_point in enumerate(transformed_source_points):
            # Find the closest point in the target cloud
            distance, index = tree.query(source_point)
            distances[i] = distance

            # Check if the distance is less than the threshold
            if distance > max_distance_correspondence:
                weight_matrices[i] = np.zeros((2, 2))
                continue

            corresponding_target_points[i] = target_points[index]
            # Compute the weight matrix by combining the covariance matrices of the target and source points
            combined_cov_matrix = source_cov_matrices[i] + rotation_matrix @ target_cov_matrices[index] @ rotation_matrix.T + epsilon * np.eye(2)
            weight_matrices[i] = np.linalg.inv(combined_cov_matrix)

        # Minimize the loss function / cost function using a nonlinear conjugate gradient algorithm
        loss_function = lambda x: loss(x, source_points, corresponding_target_points, weight_matrices)
        grad_loss_function = lambda x: grad_loss(x, source_points, corresponding_target_points, weight_matrices)
        # x0 parameter: last offset -> ITERATIVE closest point
        out = fmin_cg(f=loss_function, x0=offset, fprime=grad_loss_function, disp=False, full_output=True)
        offset = out[0]
        min_loss = out[1]

        # Check for convergence
        if np.abs(last_min_loss - min_loss) < tolerance:
            print("Converged at iteration", iteration)
            break

        # Update transformation matrix
        last_min_loss = min_loss
        transformation_matrix = offset_to_transformation_matrix(offset)
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


def transform_points(points, translation, angle):
    rot_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    return np.dot(points, rot_matrix.T) + translation


# Visualization with Pygame
def visualize_icp(source_cloud, target_cloud, all_transformations, source_cov_matrices, target_cov_matrices):
    pygame.init()
    screen = pygame.display.set_mode((1000, 700), pygame.RESIZABLE)
    clock = pygame.time.Clock()
    running = True
    step = 0

    black = (0, 0, 0)
    white = (255, 255, 255)
    red = (255, 0, 0)
    blue = (0, 0, 255)
    green = (0, 255, 0)
    yellow = (255, 255, 0)


    def draw_points(points, color):
        for point in points:
            pygame.draw.circle(screen, color, point.astype(int), 3)

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

        # Create the ellipse surface
        ell = pygame.Surface((width, height), pygame.SRCALPHA)
        ell.set_alpha(alpha)
        pygame.draw.ellipse(ell, color, (0, 0, width, height), 2)
        
        # Rotate the ellipse surface
        ell = pygame.transform.rotate(ell, -theta_degrees)
        
        # Calculate the new position to center the rotated ellipse
        rotated_rect = ell.get_rect(center=center)
        surface.blit(ell, rotated_rect)

    show_ellipses = True

    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

        screen.fill((0, 0, 0))
        transformed_source = apply_transformation(source_cloud, all_transformations[step])

        draw_points(source_cloud, (0, 0, 255))  # Blue for original source
        draw_points(target_cloud, (0, 255, 0))  # Green for target
        draw_points(transformed_source, (255, 0, 0))  # Red for transformed source

        # Draw the last transformation matrix
        font = pygame.font.Font(None, 36)
        text = font.render("Transformation Matrix:", True, (255, 255, 255))
        screen.blit(text, (10, 10))
        for i in range(3):
            for j in range(3):
                text = font.render(f"{all_transformations[step][i, j]:.2f}", True, (255, 255, 255))
                screen.blit(text, (10 + j * 100, 50 + i * 40))
        screen.blit(font.render("Press left/right arrow keys to navigate", True, (255, 255, 255)), (400, 10))
        screen.blit(font.render("Press 'e' to toggle ellipses", True, (255, 255, 255)), (400, 50))

        # Draw the current step
        text = font.render(f"Step: {step}", True, (255, 255, 255))
        screen.blit(text, (10, 250))

        # pygame.display.flip()
        # clock.tick(1)  # Slow down for visualization
        # step = (step + 1) % len(all_transformations)

        # listen on the arrow keys to navigate through the steps (only one per click)
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            step = (step - 1) % len(all_transformations)
            pygame.time.wait(100)
        if keys[pygame.K_RIGHT]:
            step = (step + 1) % len(all_transformations)
            pygame.time.wait(100)
        if keys[pygame.K_e]:
            show_ellipses = not show_ellipses
            pygame.time.wait(100)
        

        # Draw ellipses
        if show_ellipses:
            for i, point in enumerate(source_points):
                draw_ellipse(screen, yellow, point, source_cov_matrices[i])
        
            for i, point in enumerate(target_points):
                draw_ellipse(screen, green, point, target_cov_matrices[i])

        pygame.display.update()
        clock.tick(30)


    pygame.quit()

if __name__ == "__main__":
    circle_center = (300, 150)
    circle_radius = 100
    num_points_circle = 30

    square_center = (600, 250)
    square_size = 200
    num_points_square_side = 20

    source_points_circle = generate_circle_points(circle_center, circle_radius, num_points_circle)
    source_points_square = generate_square_points(square_center, square_size, num_points_square_side)

    source_points = np.concatenate([source_points_circle, source_points_square])


    translation = np.array([150, -50])
    rotation_angle = np.pi / 6
    target_points = transform_points(source_points, translation, rotation_angle)

    # add a small noise to the points
    source_points += np.random.normal(0, 5, source_points.shape)
    target_points += np.random.normal(0, 10, target_points.shape)

    # shuffle the target points and remove some of them
    np.random.shuffle(target_points)
    target_points = target_points[:len(target_points) - 3]

    T, all_transformations, source_cov_matrices, target_cov_matrices = gicp(source_points, target_points)

    visualize_icp(source_points, target_points, all_transformations, source_cov_matrices, target_cov_matrices)