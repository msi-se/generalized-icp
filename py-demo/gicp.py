#!/usr/bin/python

"""
2D Generalized ICP

WHAT:

This is a python implementation of the generalized ICP algorithm described in:
"Generalized-ICP", Aleksandr V. Segal, Dirk Haehnel, and Sebastian Thrun, In Robotics: Science and Systems 2009

Paper available on Aleksander Segal's personal website:
http://www.stanford.edu/~avsegal/

TO INSTALL:

This code requires 3 modules to be installed on your system.  Numpy, Scipy, and the Python NIPALS PCA module by Henning Risvik.

http://www.scipy.org/kk

http://np.scipy.org/

http://folk.uio.no/henninri/pca_module/


TO RUN:

To run the example program, simply execute this file from the command-line with the following command:
python gen_icp.py

Copy the main section to your own code to have a working implementation and include this file as a module.

This code is provided under the GNU General Public License, Version 3

Copyright 2009, by Jacob Everist
jacob.everist@gmail.com

http://jacobeverist.com/gen_icp

"""

import random
import scipy
import scipy.linalg
import scipy.optimize
import math
import pylab
import numpy as np
import pygame
from pygame.locals import QUIT

from copy import copy
from copy import deepcopy

# displace the point by the offset plus modify it's covariance
def dispPoint(p, offset):
    xd = offset[0]
    yd = offset[1]
    theta = offset[2]

    T = np.matrix([    [math.cos(theta), -math.sin(theta), xd],
            [math.sin(theta), math.cos(theta), yd],
            [0.0, 0.0, 1.0]
            ])

    p_hom = np.matrix([[p[0]],[p[1]],[1.0]])
    temp = T*p_hom
    p_off = [temp[0,0],temp[1,0]]


    #R = np.matrix([    [math.cos(theta), -math.sin(theta)],
        #[math.sin(theta), math.cos(theta)] ])

    #Cv = p[2]
    #Ca = R * Cv * np.transpose(R)
    #p_off.append(Ca)

    return p_off

# displace the point by the offset only.  No covariance
def dispOffset(p, offset):
    xd = offset[0]
    yd = offset[1]
    theta = offset[2]

    T = np.matrix([    [math.cos(theta), -math.sin(theta), xd],
            [math.sin(theta), math.cos(theta), yd],
            [0.0, 0.0, 1.0]
            ])

    p_hom = np.matrix([[p[0]],[p[1]],[1.0]])
    temp = T*p_hom
    p_off = [temp[0,0],temp[1,0]]

    return p_off


def disp(ai, bi, T):

    temp = T*ai
    result = bi-temp

    result[2] = 1.0

    return result


def computeMatchError(offset, a, b, Ca, Cb):

    xd = offset[0]
    yd = offset[1]
    theta = offset[2]

    T = np.matrix([    [math.cos(theta), -math.sin(theta), xd],
            [math.sin(theta), math.cos(theta), yd],
            [0.0, 0.0, 1.0]
            ])

    R = np.matrix([    [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)] ])

    Cv = Ca

    d_vec = disp(np.concatenate((a,np.matrix([1.0]))), np.concatenate((b,np.matrix([1.0]))), T)

    res = Cb + R * Cv * np.transpose(R)

    # remove the 3rd homogeneous dimension for inverse
    invMat = scipy.linalg.inv(res)

    # add homogeneous dimension back
    invMat = np.concatenate((invMat,np.matrix([[0.0],[0.0]])), 1)
    invMat = np.concatenate((invMat,np.matrix([0.0,0.0,0.0])))

    error = np.transpose(d_vec)*invMat*d_vec
    errVal = error[0,0]

    return errVal

def nipals(X, n_components=2, tol=1e-6, max_iter=1000):
    X = X - np.mean(X, axis=0)  # Centering
    scores = np.zeros((X.shape[0], n_components))
    loadings = np.zeros((X.shape[1], n_components))
    for i in range(n_components):
        t = X[:, i]  # Initial guess for scores
        for j in range(max_iter):
            p = X.T @ t / (t.T @ t)
            p = p / np.linalg.norm(p)
            t_new = X @ p / (p.T @ p)
            if np.linalg.norm(t - t_new) < tol:
                break
            t = t_new
        scores[:, i] = t
        loadings[:, i] = p
        X = X - np.outer(t, p)
    return scores, loadings

def findLocalNormal(pnt,points):

    x_list = []
    y_list = []

    pnt_count = 0
    pnts_copy = deepcopy(points)

    while pnt_count < 10:
        # select 3 closest points
        minDist = 1e100
        minPoint = []
        for p in pnts_copy:
            dist = math.sqrt((p[0]-pnt[0])**2 + (p[1]-pnt[1])**2)

            if dist < minDist:
                minDist = dist
                minPoint = p

        x_list.append(minPoint[0])
        y_list.append(minPoint[1])

        pnts_copy.remove(minPoint)
        pnt_count += 1

    x_list.append(pnt[0])
    y_list.append(pnt[1])

    cov_a = scipy.cov(x_list,y_list)

    loadings = []

    # NOTE:  seems to create opposing colinear vectors if data is colinear, not orthogonal vectors

    try:
        scores, loadings, E = nipals(cov_a, 2, 0.000001)

    except:
        raise

    if len(loadings) < 2:
        raise

    # return the second vector returned from PCA because this has the least variance (orthogonal to plane)
    return loadings[1]

def computeVectorCovariance(vec,x_var,y_var):
    Cv = np.matrix([    [x_var, 0.0],
            [0.0, y_var]
            ])

    mag = math.sqrt(vec[0]**2 + vec[1]**2)
    normVec = [vec[0]/mag, vec[1]/mag]

    if normVec[1] == 0:
        R = np.matrix([    [1.0, 0.0],
                [0.0, 1.0]
                ])

    else:
        B = -1 / (normVec[1] + normVec[0]**2/normVec[1])
        A = -normVec[0]*B/normVec[1]
        R = np.matrix([    [A, -B],
                [B, A]
                ])

    Ca = np.transpose(R) * Cv * R

    return Ca

# for point T*a, find the closest point b in B
def findClosestPointInB(b_data, a, offset):

    xd = offset[0]
    yd = offset[1]
    theta = offset[2]

    T = np.matrix([    [math.cos(theta), -math.sin(theta), xd],
            [math.sin(theta), math.cos(theta), yd],
            [0.0, 0.0, 1.0]
            ])


    a_hom = np.matrix([[a[0]],[a[1]],[1.0]])
    temp = T*a_hom
    a_off = [temp[0,0],temp[1,0]]

    minDist = 1e100
    minPoint = None

    for p in b_data:

        dist = math.sqrt((p[0]-a_off[0])**2 + (p[1]-a_off[1])**2)
        if dist < minDist:
            minPoint = copy(p)
            minDist = dist


    if minPoint != None:
        return minPoint, minDist
    else:
        raise


def cost_func(offset, match_pairs):

    sum = 0.0
    for pair in match_pairs:

                a = np.matrix([[pair[0][0]],[pair[0][1]]])
                b = np.matrix([[pair[1][0]],[pair[1][1]]])
                sum += computeMatchError(offset, a, b, pair[2], pair[3])

        # NOTE:  standard point-to-point ICP
                #a = np.matrix([[pair[0][0]],[pair[0][1]],[1.0]])
                #b = np.matrix([[pair[1][0]],[pair[1][1]],[1.0]])

                #distVec = disp(a, b, T)
                #mag = distVec[0,0]**2 + distVec[1,0]**2
                #sum += mag

    return sum

def precomputeCovariance(points, high_var=1.0, low_var=0.001):

    for p in points:
        C = np.matrix([    [high_var, 0.0],
                [0.0, high_var]
                ])

        try:
            # the covariance matrix that enforces the point-to-plane constraint
            normVec = findLocalNormal(p,points)
            C = computeVectorCovariance(normVec,low_var,high_var)

        except:
            pass

        p.append(C)

def gen_ICP(offset, a_data, b_data, costThresh = 0.004, minMatchDist = 2.0):

    # 1. compute the local line for each point
    # 2. compute the covariance for point-to-line constraint
    # 3. find closest points in A of B
    # 4. discard matches beyond threshold d_max < | ai - T *bi |
    # 5. optimize T for the sum of computeMatchError
    # 6. if converged, stop, else go to 3 with b offset by new T

    # DONE:  store the covariance for each point untransformed so we don't have to repeat
    # DONE:  transform each covariance by the appropriate rotation at each point

    # compute the local covariance matrix for a point-to-plane contrainst
    precomputeCovariance(a_data)
    precomputeCovariance(b_data)

    numIterations = 0
    lastCost = 1e100

    all_offsets = [offset]

    while True:

        a_trans = []

        # pre-transform the A points and their associated covariances
        for p in a_data:
            a_trans.append(dispPoint(p, offset))

        match_pairs = []
        for i in range(len(a_trans)):
            a_p = a_trans[i]

            # for every transformed point of A, find it's closest neighbor in B
            b_p, minDist = findClosestPointInB(b_data, a_p, [0.0,0.0,0.0])

            if minDist <= minMatchDist:

                # add to the list of match pairs less than 1.0 distance apart
                # keep A points and covariances untransformed
                Ca = a_data[i][2]

                Cb = b_p[2]

                # we store the untransformed point, but the transformed covariance of the A point
                match_pairs.append([a_data[i],b_p,Ca,Cb])


        # optimize the match error for the current list of match pairs
        newOffset = scipy.optimize.fmin(cost_func, offset, (match_pairs,))

        # get the current cost
        newCost = cost_func(newOffset, match_pairs)

        # check for convergence condition, different between last and current cost is below threshold
        if abs(lastCost - newCost) < costThresh:
            offset = newOffset
            lastCost = newCost
            break

        # save the current offset and cost
        offset = newOffset
        lastCost = newCost
        numIterations += 1

        all_offsets.append(offset)

    return offset, all_offsets

def heatmappedPoints(data):
        #data = [[0,0],[1,0],[2,0]]

        bins = 100
        xmin = -5
        xmax = 5
        step = (xmax-xmin)*1.0 / bins
        x_edges = [ xmin+i*step for i in range(bins)]
        heatmap = np.zeros((bins,bins),dtype ='int')

        for p in data:
            i = (p[0]-xmin)/step
            j = (p[1]-xmin)/step
            if i < bins and i >=0 and j < bins and j>= 0:
                heatmap[i][j] += 1

        hist_points = []
        for i in range(bins):
            for j in range(bins):
                if heatmap[i][j] > 0.0:
                    hist_points.append([round(x_edges[i],3),
                                        round(x_edges[j],3)])

        return hist_points

def heatmap(data):
    bins = 50
    xmin = -20
    xmax = 20
    step = (xmax-xmin)*1.0 / bins
    x_edges = [ xmin+i*step for i in range(bins)]
    heatmap = np.zeros((bins,bins),dtype ='int')

    for p in data:
        i = (p[0]-xmin)/step
        j = (p[1]-xmin)/step
        if i < bins and i >=0 and j < bins and j>= 0:
            heatmap[j][i] += 1

    return heatmap


def EuclideanDist(m1, m2):
    # m1 and m2 have the same shape
    shape = m1.shape
    dist = 0
    for i in range(shape[0]):
        for j in range(shape[1]):
            dist += (m1[i][j]-m2[i][j])**2
    return dist

def alignFlip(data, target):
    def flip_x(m):
        return np.fliplr(m)

    def flip_y(m):
        return np.flipud(m)

    def flip_xy(m):
        return np.flipud(np.fliplr(m))

    m1 = heatmap(data)
    m2 = heatmap(target)
    m1_x_flip = flip_x(m1)
    m1_y_flip = flip_y(m1)
    m1_xy_flip = flip_xy(m1)

    dist_original = EuclideanDist(m1, m2)
    dist_x_flip = EuclideanDist(m1_x_flip, m2)
    dist_y_flip = EuclideanDist(m1_y_flip, m2)
    dist_xy_flip = EuclideanDist(m1_xy_flip, m2)

    closest_m = None
    mindist = 1e+100
    for m, d in [(m1, dist_original),(m1_x_flip, dist_x_flip),
                 (m1_y_flip, dist_y_flip),(m1_xy_flip, dist_xy_flip)]:
        if d < mindist:
            mindist = d
            closest_m = m

    # return points
    #bins = 100
    #xmin = -5
    #xmax = 5
    #step = (xmax-xmin)*1.0 / bins
    #x_edges = [ xmin+i*step for i in range(bins)]
    #hist_points = []
    #for i in range(bins):
        #for j in range(bins):
            #if closest_m[i][j] > 0.0:
                #hist_points.append([round(x_edges[i],3),
                                    #round(x_edges[j],3)])
    result = closest_m.tolist()

    return closest_m


def align(data, target):
    if len(target) == 0:
        return heatmap(data)
    else:
        return alignFlip(data, target)
    #return heatmappedPoints(data)
    #data = heatmappedPoints(data)
    #target = heatmappedPoints(target)
    ## TUNE ME:  threshold cost difference between iterations to determine if converged
    #costThresh = 0.04

    ## TUNE ME:   minimum match distance before point is discarded from consideration
    #minMatchDist = 2.0

    ## plot the best fit at each iteration of the algorithm?
    #plotIteration = False

    #offset = [0.0,0.0,-math.pi/4]

    #offset = gen_ICP(offset, data, target,
            #costThresh, minMatchDist, plotIteration)

    #data_trans = []
    #for p in data:
        #data_trans.append(dispPoint(p, offset))

    #return data_trans

# Function to generate square points and normals
def generate_square_points(center, size, num_points_per_side):
    points = []
    half_size = size / 2
    sides = [np.linspace(center[0] - half_size, center[0] + half_size, num_points_per_side),
             np.linspace(center[1] - half_size, center[1] + half_size, num_points_per_side)]
    
    # Bottom side
    for x in sides[0]:
        points.append([x, center[1] - half_size])
    
    # Right side
    for y in sides[1]:
        points.append([center[0] + half_size, y])
    
    # Top side
    for x in sides[0]:
        points.append([x, center[1] + half_size])
    
    # Left side
    for y in sides[1]:
        points.append([center[0] - half_size, y])
    
    # random remove half of the points
    indices = random.sample(range(len(points)), len(points) // 2)
    points = [points[i] for i in range(len(points)) if i in indices]
    
    return points

def draw_points(points, color):
    for point in points:
        pygame.draw.circle(screen, color, point, 3)

def are_same_matrix(m1, m2, tol=1e-6):
    if m1.shape != m2.shape:
        return False
    for i in range(m1.shape[0]):
        if not np.allclose(m1[i], m2[i], atol=tol):
            return False
    return True


def offset_to_transformation_matrix(offset):
    xd = offset[0]
    yd = offset[1]
    theta = offset[2]

    T = np.matrix([    [math.cos(theta), -math.sin(theta), xd],
            [math.sin(theta), math.cos(theta), yd],
            [0.0, 0.0, 1.0]
            ])

    return T

if __name__ == '__main__':

    # sample data
    square_center = (400, 250)
    square_size = 200
    num_points_square_side = 10

    # a_data = [[1.0,1.0],[1.1,1.1],[1.2,1.2],[1.3,1.31],[1.4,1.4],[1.51,1.5],[1.6,1.6],[1.7,1.7]]
    # b_data = [[0.3,1.0],[0.3,1.1],[0.3,1.2],[0.31,1.3],[0.3,1.4],[0.3,1.5],[0.3,1.6]]
    a_data = generate_square_points(square_center, square_size, num_points_square_side)

    b_data_offset = [50, 50, np.pi / 6]
    b_data = [dispPoint(p, b_data_offset) for p in a_data]

    # run generalized ICP (a plot is made for each iteration of the algorithm)
    # deep copy the data so we can keep the original data
    copy_a = deepcopy(a_data)
    copy_b = deepcopy(b_data)
    offset, all_offsets = gen_ICP([0, 0, 0], copy_a, copy_b, costThresh=1, minMatchDist=100)

    # get the transformation matrix
    T = offset_to_transformation_matrix(offset)
    T_expected = offset_to_transformation_matrix(b_data_offset)

    print("Expected transformation matrix:")
    print(T_expected)
    print("Actual transformation matrix:")
    print(T)

    # check if the transformation matrix is correct
    if are_same_matrix(T, T_expected, tol=1e-7):
        print("Transformation matrix is correct")
    else:
        raise Exception("Transformation matrix is incorrect")

    quit()

    pygame.init()
    width, height = 800, 600
    screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
    pygame.display.set_caption("GICP Algorithm Visualization")

    black = (0, 0, 0)
    white = (255, 255, 255)
    red = (255, 0, 0)
    blue = (0, 0, 255)

    running = True
    clock = pygame.time.Clock()

    step = 0
    max_steps = len(all_offsets)
    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

        screen.fill(black)
        
        if step < max_steps:
            current_offset = all_offsets[step]
            transformed_source = [dispPoint(p, current_offset) for p in a_data]
            step += 1
        draw_points(a_data, white)
        draw_points(b_data, blue)
        draw_points(transformed_source, red)
        
        pygame.display.flip()
        clock.tick(2)  # Control the speed of the visualization

    pygame.quit()