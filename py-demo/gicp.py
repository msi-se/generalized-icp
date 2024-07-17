import numpy as np
from scipy.spatial import KDTree
import random
import pygame
from pygame.locals import QUIT
from scipy.optimize import fmin_cg

def compute_covariance_matrix_single_point(neighbors, epsilon=100):
    # "In our implementation we compute these transformations by considering
    # the eigen decomposition of the empirical covariance of the 20 closest points,
    # Σ = UDUT . We then use U in place of the rotation matrix (in effect
    # replacing D with diag(ǫ, 1, 1) to get the ﬁnal surface-aligned matrix)." (GICP from Segal et al.)

    covariance_ground = np.array([[epsilon, 0], [0, epsilon*0.1]])
    covariance = np.cov(neighbors, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eig(covariance)
    max_eigenvector = eigenvectors[:, np.argmax(eigenvalues)] # eigenvector with the highest eigenvalue
    rotation_matrix = np.array([[max_eigenvector[0], -max_eigenvector[1]], [max_eigenvector[1], max_eigenvector[0]]])
    covariance_aligned = rotation_matrix @ covariance_ground @ rotation_matrix.T
    return covariance_aligned

def compute_covariance_matrix(points, max_distance=40):
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
                cov_matrices[i] = compute_covariance_matrix_single_point(neighbors)
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

def gicp(source_points, target_points, max_iterations=100, tolerance=1e-6, max_distance_correspondence=150, max_distance_nearest_neighbors=50):
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
    GICP algorithm from Segal et al. (2009)
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
    highest_weight_points_source = []
    highest_weight_points_target = []
    all_source_cov_matrices = []

    for iteration in range(max_iterations):

        # Apply the current transformation to the source points
        transformed_source_points = apply_transformation(source_points, transformation_matrix)
        source_cov_matrices = compute_covariance_matrix(transformed_source_points, max_distance_nearest_neighbors)
        all_source_cov_matrices.append(source_cov_matrices)
        
        # Compute the corresponding target points and weight matrices
        corresponding_target_points = np.zeros_like(source_points)
        weight_matrices = np.zeros((len(source_points), 2, 2))
        combined_cov_matrices = np.zeros((len(source_points), 2, 2))
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
            combined_cov_matrices[i] = combined_cov_matrix
            weight_matrices[i] = np.linalg.inv(combined_cov_matrix)

        # Minimize the loss function / cost function using maximum likelihood estimation
        loss_function = lambda x: loss(x, source_points, corresponding_target_points, weight_matrices)
        grad_loss_function = lambda x: grad_loss(x, source_points, corresponding_target_points, weight_matrices)
        
        # x0 parameter: last offset -> "ITERATIVE" closest point
        out = fmin_cg(f=loss_function, x0=offset, fprime=grad_loss_function, disp=False, full_output=True)
        offset = out[0]
        min_loss = out[1]
        delta_loss = np.abs(last_min_loss - min_loss)

        # print(f"Loss: {min_loss:16.10f}, Delta Loss: {delta_loss:16.10f}, Iteration: {iteration:3d}, Distances: {np.mean(distances):9.4f}, Offset: trans: { offset[0]:9.4f}, { offset[1]:9.4f}, rot: { offset[2]:9.4f}")

        # Check for convergence
        if delta_loss < tolerance:
            print("Converged at iteration", iteration)
            break

        # Update transformation matrix
        last_min_loss = min_loss
        transformation_matrix = offset_to_transformation_matrix(offset)
        all_transformations.append(transformation_matrix)

        # only for visualization: store the 5 highest weight matrices and their corresponding points
        highest_weight_indices = np.argsort(np.linalg.det(weight_matrices))[-5:]
        highest_weight_points_source.append(transformed_source_points[highest_weight_indices])
        highest_weight_points_target.append(corresponding_target_points[highest_weight_indices])

    return transformation_matrix, all_transformations, initial_source_cov_matrices, target_cov_matrices, highest_weight_points_source, highest_weight_points_target, all_source_cov_matrices

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
def visualize_icp(source_cloud, target_cloud, all_transformations, source_cov_matrices, target_cov_matrices, highest_weight_points_source, highest_weight_points_target):
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
    show_highest_cov = True

    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                # listen on the arrow keys to navigate through the steps (only one per click)
                if event.key == pygame.K_LEFT:
                    step = (step - 1) % len(all_transformations)
                if event.key == pygame.K_RIGHT:
                    step = (step + 1) % len(all_transformations)
                if event.key == pygame.K_e:
                    show_ellipses = not show_ellipses
                if event.key == pygame.K_c:
                    show_highest_cov = not show_highest_cov
                if event.key == pygame.K_ESCAPE:
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

        # mark the 5 highest weight matrices with a circle around them
        if show_highest_cov:
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]
            for i in range(5):
                try:
                    pygame.draw.circle(screen, colors[i], highest_weight_points_source[step][i].astype(int), 10, 2)
                    pygame.draw.circle(screen, colors[i], highest_weight_points_target[step][i].astype(int), 10, 2)
                except IndexError:
                    pass

        # Draw ellipses
        if show_ellipses:
            for i, point in enumerate(transformed_source):
                draw_ellipse(screen, yellow, point, source_cov_matrices[step][i])
        
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

    T, all_transformations, _, target_cov_matrices, highest_weight_points_source, highest_weight_points_target, source_cov_matrices = gicp(source_points, target_points)

    visualize_icp(source_points, target_points, all_transformations, source_cov_matrices, target_cov_matrices, highest_weight_points_source, highest_weight_points_target)