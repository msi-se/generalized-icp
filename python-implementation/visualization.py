import numpy as np
from scipy.spatial import KDTree
import random
import pygame
from pygame.locals import QUIT
from scipy.optimize import fmin_cg
from gicp import gicp, apply_transformation

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

    show_ellipses = 1
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
                    show_ellipses += 1
                    show_ellipses %= 3
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
        if show_ellipses != 0:
            if show_ellipses == 1:
                for i, point in enumerate(source_points):
                    draw_ellipse(screen, yellow, point, source_cov_matrices[0][i])
            
            if show_ellipses == 2:
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