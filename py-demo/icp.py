import random
import numpy as np
import pygame
from pygame.locals import QUIT
from scipy.spatial import cKDTree

def icp(source_cloud, target_cloud, max_iterations=50, tolerance=1e-5):
    T = np.eye(3)
    all_transformations = [T]
    all_correspondences = [[]]

    for _ in range(max_iterations):
        correspondences = find_correspondences(source_cloud, target_cloud, T)
        all_correspondences.append(correspondences)
        T_new = compute_transformation(correspondences)
        T = T_new @ T  # Correct update by matrix multiplication
        all_transformations.append(T)

        # check for convergence
        if np.linalg.norm(T_new - np.eye(3)) < tolerance:
            break

    return T, all_transformations, all_correspondences

def find_correspondences(source_cloud, target_cloud, T):
    transformed_source = apply_transformation(source_cloud, T)
    correspondences = []
    for p_s in transformed_source:
        closest_point = find_closest_point(p_s, target_cloud)
        correspondences.append((p_s, closest_point))
    return correspondences

def find_closest_point(point, target_cloud):
    tree = cKDTree(target_cloud[:, :2])
    _, min_index = tree.query(point[:2], k=1)
    return target_cloud[min_index]

def apply_transformation(cloud, T):
    homogeneous_cloud = np.hstack((cloud[:, :2], np.ones((cloud.shape[0], 1))))
    transformed_cloud = (T @ homogeneous_cloud.T).T
    transformed_cloud = np.hstack((transformed_cloud[:, :2], cloud[:, 2:]))
    return transformed_cloud

def apply_transformations(cloud, transformations):
    transformed_cloud = cloud.copy()
    for T in transformations:
        transformed_cloud = apply_transformation(transformed_cloud, T)
    return transformed_cloud

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

# Visualization with Pygame
def visualize_icp(source_cloud, target_cloud, all_transformations, all_correspondences):
    pygame.init()
    width, height = 1000, 800
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()
    running = True
    step = 0

    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

        screen.fill((0, 0, 0))
        transformed_source = apply_transformation(source_cloud, all_transformations[step])

        draw_points(screen, source_cloud, (0, 0, 255))  # Blue for original source
        draw_points(screen, target_cloud, (0, 255, 0))  # Green for target
        draw_points(screen, transformed_source, (255, 0, 0))  # Red for transformed source

        # Draw correspondences (as lines)
        try:
            for (p_s, p_t) in all_correspondences[step + 1]:
                draw_line(screen, p_s, p_t, (255, 255, 255))
        except IndexError:
            pass

        # Draw the last transformation matrix
        font = pygame.font.Font(None, 36)
        text = font.render("Transformation Matrix:", True, (255, 255, 255))
        screen.blit(text, (10, 10))
        for i in range(3):
            for j in range(3):
                text = font.render(f"{all_transformations[step][i, j]:.2f}", True, (255, 255, 255))
                screen.blit(text, (10 + j * 100, 50 + i * 40))

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

        pygame.display.update()
        clock.tick(30)


    pygame.quit()

def draw_points(screen, cloud, color):
    for point in cloud:
        pygame.draw.circle(screen, color, (int(point[0] * 100 + 200), int(-point[1] * 100 + 600)), 5)

def draw_line(screen, start, end, color):
    pygame.draw.line(screen, color, (int(start[0] * 100 + 200), int(-start[1] * 100 + 600)),
                     (int(end[0] * 100 + 200), int(-end[1] * 100 + 600)))

def transformation_matrix_from_translation_and_rotation(translation, rotation):
    T = np.eye(3)
    T[0, 2] = translation[0]
    T[1, 2] = translation[1]
    T[0, 0] = np.cos(rotation)
    T[0, 1] = -np.sin(rotation)
    T[1, 0] = np.sin(rotation)
    T[1, 1] = np.cos(rotation)
    return T

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

def main():
    # Example 2D data: random points
    np.random.seed(42)
    (source_cloud, n) = generate_square_points([0, 0], 2, 20)

    # create a target cloud by shifting the source cloud and rotating it (but just a small amount)
    # and apply some noise
    translation = np.array([4, 0.5])
    rotation_angle = np.pi / 12 # 15 degrees
    T = transformation_matrix_from_translation_and_rotation(translation, rotation_angle)
    target_cloud = apply_transformation(source_cloud, T)
    target_cloud += np.random.normal(0, 0.03, target_cloud.shape)

    max_iterations = 50
    tolerance = 1e-7
    T, all_transformations, all_correspondences = icp(source_cloud, target_cloud, max_iterations, tolerance)

    for i, T_i in enumerate(all_transformations):
        print(f"Transformation {i}:")
        print(f"{T_i[0, 0]:.2f} {T_i[0, 1]:.2f} {T_i[0, 2]:.2f}")
        print(f"{T_i[1, 0]:.2f} {T_i[1, 1]:.2f} {T_i[1, 2]:.2f}")
        print(f"{T_i[2, 0]:.2f} {T_i[2, 1]:.2f} {T_i[2, 2]:.2f}")
        print()

    print("Estimated Transformation Matrix:")
    print(T)

    visualize_icp(source_cloud, target_cloud, all_transformations, all_correspondences)

if __name__ == "__main__":
    main()