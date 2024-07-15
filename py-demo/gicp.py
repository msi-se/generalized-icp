import numpy as np
from scipy.spatial import KDTree
import random
import pygame
from pygame.locals import QUIT
from scipy.optimize import fmin_cg
from scipy.optimize import minimize
from scipy.optimize import newton

def compute_covariance_matrix(points, max_distance=30):
    """Find the k nearest neighbors of each point in the cloud and compute the covariance matrices with them."""
    tree = KDTree(points)
    cov_matrices = np.zeros((len(points), 2, 2))
    for i in range(len(points)):
        distances, indices = tree.query(points[i], k=6, distance_upper_bound=max_distance)
        indices = np.delete(indices, np.where(indices == len(points)))
        # Ensure there are enough neighbors for covariance calculation
        if len(indices) > 1:
            neighbors = points[indices]
            try:
                cov_matrices[i] = np.cov(neighbors.T)
            except np.linalg.LinAlgError:
                cov_matrices[i] = np.eye(2)
        else:
            cov_matrices[i] = np.eye(2)
    return cov_matrices

def rot_mat(theta):
    """Return a 2D rotation matrix given the angle theta."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])

def offset_to_transformation_matrix(offset):
    """Convert the offset parameters (tx, ty, theta) to a transformation matrix."""
    translation = offset[:2]
    theta = offset[2]
    rotation_matrix = rot_mat(theta)
    transformation_matrix = np.eye(3)
    transformation_matrix[:2, :2] = rotation_matrix
    transformation_matrix[:2, 2] = translation
    return transformation_matrix

def loss(offset, source_points, target_points, weight_matrices):
    """Compute the loss function for the given transformation parameters."""
    # $d_i^((T)) arrow.l b_i - T dot.op m_i$
    # $T arrow.l arg min_T { sum_i d_i^(T)^T (C_i^B + T C_i^A T^T)^(-1) d_i^((T)) }$  // Maximum Likelihood Estimation

    def loss_i(i):
        d = target_points[i] - transform_points(source_points[i], offset[:2], offset[2])
        return np.dot(d, np.dot(weight_matrices[i], d))
    
    return np.sum([loss_i(i) for i in range(len(source_points))])

def gicp(source_points, target_points, max_iterations=100, tolerance=1e-6, epsilon=1e-6, max_distance_correspondence=150, max_distance_nearest_neighbors=50):
    """
    + $T arrow.l T_0$
    + *while* not converged *do*
      + *for* $i arrow.l 1$ *to* $N$ *do*
        + $m_i arrow.l$ `FindClosestPointInA`$(T dot.op b_i)$
        + $d_i^((T)) arrow.l b_i - T dot.op m_i$      // Residuum / Abstand
        + *if* $parallel d_i^((T)) parallel lt.eq d_(max)$ *then*
          + $C_i^A arrow.l$ `computeCovarianceMatrix`$(T dot.op b_i)$
          + $C_i^B arrow.l$ `computeCovarianceMatrix`$(m_i)$
        + *else*
          + $C_i^A arrow.l 0$; #h(1em) $C_i^B arrow.l 0$
        + *end*
      + *end*
      + $T arrow.l arg min_T {
            sum_i d_i^(T)^T (C_i^B + T C_i^A T^T)^(-1) d_i^((T))
          }$  // Maximum Likelihood Estimation
    + *end*

    """

    # Convert points to numpy arrays if they are not already
    source_points = np.asarray(source_points)
    target_points = np.asarray(target_points)

    # Compute nearest neighbors and covariance matrices of the target points
    target_cov_matrices = compute_covariance_matrix(target_points, max_distance_nearest_neighbors)

    # Initialize transformation matrix and offset (is the same but offset needed for fmin_cg)
    transformation_matrix = np.eye(3)
    all_transformations = [transformation_matrix]
    offset = np.zeros(3)  # [tx, ty, theta]
    last_min_loss = np.inf
    initial_source_cov_matrices = compute_covariance_matrix(source_points, max_distance_nearest_neighbors)

    for iteration in range(max_iterations):

        # Apply the current transformation to the source points
        transformed_source_points = apply_transformation(source_points, transformation_matrix)
        source_cov_matrices = compute_covariance_matrix(transformed_source_points, max_distance_nearest_neighbors)
        
        # Compute the corresponding target points and weight matrices
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

            # Compute the weight matrix by combining the covariance matrices of the target and source points (target = A; source = B)
            combined_cov_matrix = source_cov_matrices[i] + target_cov_matrices[index]
            weight_matrices[i] = np.linalg.inv(combined_cov_matrix)

        # Minimize the loss function / cost function using maximum likelihood estimation
        loss_function = lambda x: loss(x, source_points, corresponding_target_points, weight_matrices)

        # x0 parameter: last offset -> "ITERATIVE" closest point
        out = minimize(loss_function, x0=offset, method='L-BFGS-B', tol=epsilon) # L-BFGS-B: Limited-memory Broyden-Fletcher-Goldfarb-Shanno algorithm
        offset = out["x"]
        min_loss = out["fun"]
        delta_loss = np.abs(last_min_loss - min_loss)

        print(f"Loss: {min_loss:16.10f}, Delta Loss: {delta_loss:16.10f}, Iteration: {iteration:3d}, Distances: {np.mean(distances):9.4f}, Offset: trans: { offset[0]:9.4f}, { offset[1]:9.4f}, rot: { offset[2]:9.4f}")

        # Check for convergence
        if delta_loss < tolerance:
            print("Converged at iteration", iteration)
            break


        # Update transformation matrix
        last_min_loss = min_loss
        transformation_matrix = offset_to_transformation_matrix(offset)
        all_transformations.append(transformation_matrix)

    return transformation_matrix, all_transformations, initial_source_cov_matrices, target_cov_matrices

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
    num_points_square_side = 30

    source_points_circle = generate_circle_points(circle_center, circle_radius, num_points_circle)
    source_points_square = generate_square_points(square_center, square_size, num_points_square_side)

    source_points = np.concatenate([source_points_circle, source_points_square])


    translation = np.array([150, -50])
    rotation_angle = np.pi / 3
    target_points = transform_points(source_points, translation, rotation_angle)

    # add a small noise to the points
    source_points += np.random.normal(0, 2, source_points.shape)
    target_points += np.random.normal(0, 5, target_points.shape)

    # shuffle the target points and remove some of them
    np.random.shuffle(target_points)
    target_points = target_points[:len(target_points) - 3]

    T, all_transformations, source_cov_matrices, target_cov_matrices = gicp(source_points, target_points)

    visualize_icp(source_points, target_points, all_transformations, source_cov_matrices, target_cov_matrices)