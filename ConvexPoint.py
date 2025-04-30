from scipy.spatial import ConvexHull, convex_hull_plot_2d
import numpy as np
from scipy.spatial import Delaunay
from matplotlib import pyplot as plt
import math
import os
import shutil as shutil

from sklearn.neighbors import KDTree

from File import SaveFile
from SurfaceFitting import Fit
import test


# THIS NEEDS TO MODIFY, CANNOT BE USED FOR NOW

def ProjectTo2D(AllClusterList, AllClusterNormalList, part, GridPoint_dist = 1):
    # This is for Main.py

    # Let see the cluster in each layer
    for layer in range(0, len(AllClusterList)):
        print('layer: ', layer)
        ColoredClusterEdge = []
        AllGrid = []
        CutGridThreeDPoints = []
        AllEdgePoints = []
        NonColorAllEdgePoints = []

        # Every layer has own cluster
        for cluster in range(0, len(AllClusterList[layer])):
            # Preparation for Clustering
            # All points will be  projected using same normal direction
            # print('layer no:', layer)

            # print('We have', len(AllClusterList[layer]), 'cluster')
            # print('cluster no:', cluster)
            #
            # print('We have', len(AllClusterList[layer][cluster]), 'points')
            # print('We have', len(AllClusterNormalList[layer][cluster]), 'N points')
            # print('We have', len(AllClusterNormalList[layer][2]), 'CL points')
            # print(AllClusterNormalList[layer][cluster])
            # print(AllClusterNormalList[layer][cluster][0])

            Normal = AllClusterNormalList[layer][cluster][0]
            Unit_N = Normal / (np.sqrt(Normal[0] * Normal[0] + Normal[1] * Normal[1] + Normal[2] * Normal[2]))

            # X coordinate
            x = [1, 0, 0]
            x = x - np.dot(x, Unit_N) * Unit_N
            x = x / (np.sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]))

            # Y coordinate
            y = np.cross(Unit_N, x)


            # Step 1 Project all points in each cluster to 2D
            Points = AllClusterList[layer][cluster]

            # convex hull needs at least 3 points
            if len(Points) > 5:
                TwoDPoints = []
                # Projecting the 3D to 2D
                for Point_no in range(0, len(Points)):
                    TwoDPoints.append([np.dot(Points[Point_no], x), np.dot(Points[Point_no], y)])

                # SaveFile('Cluster {}'.format(cluster), Points)
                # print('Cluster:', cluster)
                # print(TwoDPoints)

                TwoDPoints = np.array(TwoDPoints)
                #Show the 2D Plot
                # x, y = TwoDPoints.T
                # plt.scatter(x, y)
                # plt.show()

                # Find the boundary points
                # EdgePointIndex, EdgePoints2D = Convex(TwoDPoints)
                # print('Convex Edge Points Index\n', EdgePointIndex)

                '# Larger alpha, larger convex line'
                EdgePointIndex, EdgePoints2D = alpha_shape(TwoDPoints, alpha=9, only_outer=True) #0.25
                # print('Alpha Edge Points Index\n', EdgePointIndex)
                # print(TwoDPoints[208])
                # print(EdgePoints2D[1])


                # print('EdgePoints2D\n', EdgePoints2D)

                # Step 2 Surface Fitting
                '#Griding the points'
                Grid = Fit(Points, GridPoint_dist = GridPoint_dist)
                for n in range(0, len(Grid)):
                    AllGrid.append(Grid[n])
                Grid = np.array(Grid)

                # print('Grid\n', Grid)

                # SaveFile('Cluster {}'.format(cluster), Points)
                # SaveFile('Grid {}'.format(cluster), Grid)

                # # Step 3 coba coba
                # sample_point = np.asarray(Points)
                # tree = KDTree(sample_point, leaf_size=8)
                # dist, ind = tree.query(Grid, k=4)

                # Step 3 Remove the surface fitting point outside the boundary

                '# This 3D Edge Point, we label point as edge based on EdgePointIndex'
                EdgePoint = []
                for i in range(0, len(Points)):
                    if i in EdgePointIndex:
                        # print('i=',i)
                        EdgePoint.append(Points[i])


                '# We just make it become numpy array'
                EdgePoints2D = np.array(EdgePoints2D) #hidupin
                # print('EdgePoints2D\n', EdgePoints2D)
                # x1, y1 = EdgePoints2D.T
                # plt.scatter(x1, y1)
                # for i in range(0, len(x1)):
                #     plt.annotate(i,(x1[i],y1[i]))
                #     # ax.annotate(i, (z[i], y[i]))
                # plt.show()



                '# Projecting 3D grid to 2D grid'
                GridTwoDPoints = []
                for GridNo in range(0, len(Grid)):
                    GridTwoDPoints.append([np.dot(Grid[GridNo], x), np.dot(Grid[GridNo], y)])

                '# numpy the 2D Grid'
                GridTwoDPoints = np.array(GridTwoDPoints)
                # print('GridTwoDPoints\n', GridTwoDPoints)
                # x1, y1 = GridTwoDPoints.T
                # plt.scatter(x1, y1)
                # plt.show()

                '# Check whether the 2D Grid points are inside the boundary edge'
                for GridTwoDPointsNo in range(0, len(GridTwoDPoints)):
                    # if inside_convex_polygon(GridTwoDPoints[GridTwoDPointsNo], EdgePoints2D):
                    if inside_convex_polygon2(GridTwoDPoints[GridTwoDPointsNo], EdgePoints2D):
                    # if pnpoly(GridTwoDPoints[GridTwoDPointsNo], EdgePoints2D):

                        '# Convert back to 3D Grid using index from GridTwoDPointsNo'
                        CutGridThreeDPoints.append(Grid[GridTwoDPointsNo])


                '#[OPTIONAL] Color the edge point (for showing in 3D software)'
                for i in range(0, len(Points)):
                    ColorRange = [0, 0, 0]

                    '# If points are the edge, then color it with red'
                    if i in EdgePointIndex:
                        ColorRange = [255, 0, 0]
                        AllEdgePoints.append(Points[i] + ColorRange)
                        NonColorAllEdgePoints.append(Points[i])

                    ColoredPoint = Points[i] + ColorRange
                    ColoredClusterEdge.append(ColoredPoint)


        # Deleting and Creating New "Layer" Folder

        if not os.path.exists('Output File/Colored Cluster Edge {}'.format(part)):
            print('Note: If it is error, Make sure to close the "Layer" folder first and run it again.')
            os.makedirs('Output File/Colored Cluster Edge {}'.format(part))
            os.makedirs('Output File/Edge {}'.format(part))
            os.makedirs('Output File/Grid {}'.format(part))
            os.makedirs('Output File/Cut Grid {}'.format(part))


        '# Save points'
        SaveFile('Output File/Colored Cluster Edge {}/ColoredClusterEdge Layer {}'.format(part, layer), ColoredClusterEdge)
        SaveFile('Output File/Edge {}/Edge Layer {}'.format(part, layer), AllEdgePoints)
        SaveFile('Output File/Grid {}/Grid Layer {}'.format(part, layer), AllGrid)

        '# Save Grids which are inside edges'
        CutGridThreeDPoints = CutGridThreeDPoints + NonColorAllEdgePoints
        # print(CutGridThreeDPoints)
        SaveFile('Output File/Cut Grid {}/CutGrid Layer {}'.format(part, layer), CutGridThreeDPoints)
        print('\n')

def alpha_shape(points, alpha, only_outer=True):
    """
    Compute the alpha shape (concave hull) of a set of points.
    :param points: np.array of shape (n,2) points.
    :param alpha: alpha value.
    :param only_outer: boolean value to specify if we keep only the outer border
    or also inner edges.
    :return: set of (i,j) pairs representing edges of the alpha-shape. (i,j) are
    the indices in the points array.
    """
    assert points.shape[0] > 3, "Need at least four points"

    def CheckDuplicatePoints(points):
        temp = []
        for points_no in range(0, len(points)):
            if not points[points_no] in temp:
                temp.append(points[points_no])

        return temp

    def add_edge(edges, i, j):
        """
        Add a line between the i-th and j-th points,
        if not in the list already
        """
        if [i, j] in edges or [j, i] in edges:
            # already added
            assert [j, i] in edges, "Can't go twice over same directed edge right?"
            if only_outer:
                # if both neighboring triangles are in shape, it's not a boundary edge
                edges.remove([j, i])
            return
        edges.append([i, j])


    tri = Delaunay(points)

    # edges = set()
    edges = []

    # Loop over triangles:
    # ia, ib, ic = indices of corner points of the triangle
    for ia, ib, ic in tri.vertices:
        pa = points[ia]
        pb = points[ib]
        pc = points[ic]
        # Computing radius of triangle circumcircle
        # www.mathalino.com/reviewer/derivation-of-formulas/derivation-of-formula-for-radius-of-circumcircle
        a = np.sqrt((pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2)
        b = np.sqrt((pb[0] - pc[0]) ** 2 + (pb[1] - pc[1]) ** 2)
        c = np.sqrt((pc[0] - pa[0]) ** 2 + (pc[1] - pa[1]) ** 2)
        s = (a + b + c) / 2.0
        area = np.sqrt(s * (s - a) * (s - b) * (s - c))
        circum_r = a * b * c / (4.0 * area)
        # print('circum_r:',circum_r)
        if circum_r < alpha:
            add_edge(edges, ia, ib)
            add_edge(edges, ib, ic)
            add_edge(edges, ic, ia)

    edges_point = []
    for i, j in edges:
        # print(points[[i, j], 0], points[[i, j], 1])
        #     print(points[[i], 0], points[[i], 1])
        x = points[[i], 0]
        y = points[[i], 1]
        edges_point.append([x[0], y[0]])

        x = points[[j], 0]
        y = points[[j], 1]
        edges_point.append([x[0], y[0]])

        edges_point = CheckDuplicatePoints(edges_point)

    # print('edges before', edges_point)

    '# Sort the Points by clockwise direction'
    # edges_point = ClockWiseSort(edges_point) #IF YOU WANT TO CLOCKWISE
    # print('ep:\n', edges_point)
    # edges_point_np = np.array(edges_point)
    # x1, y1 = edges_point_np.T
    # plt.scatter(x1, y1)
    # for i in range(0, len(x1)):
    #     plt.annotate(i,(x1[i],y1[i]))
    # plt.show()

    points = points.tolist()
    # print('points\n', points)

    '# Fixing the edge point idx by clockwise direction'
    edges_point_idx = []
    for edges_point_no in range (0, len(edges_point)):
        for points_no in range (0, len(points)):
            coord_check = 0
            for coord in range(0, 2):
                if edges_point[edges_point_no][coord] == points[points_no][coord]:

                    coord_check = coord_check + 1

                if coord_check == 2:
                    edges_point_idx.append(points_no)



    return edges_point_idx, edges_point

def Convex(points):

    points = np.asarray(points)
    # print('Points\n', points)

    # Do convex
    hull = ConvexHull(points)

    # Prepare the list for edge
    EdgePointsIndex = []
    EdgePoints2D = []

    for n in range(0, len(hull.vertices)):

        a = hull.vertices[n]  # Index
        EdgePointsIndex.append(a)
        c = points[hull.vertices[n]]  # Point
        EdgePoints2D.append(c)


    # for simplex in hull.simplices:
    #     # print(points[simplex, 0])
    #     # print(points[simplex, 1])
    #     # print(points[simplex])
    #
    #     # Hull give result StartPoint[c] and EndPoint[d]
    #     c = [points[simplex][0][0], points[simplex][0][1]]
    #     d = [points[simplex][1][0], points[simplex][1][1]]
    #     # print('c', c)
    #     # print('d', d)
    #
    #     # Find the point index which is edge of hull
    #     a = simplex[0]
    #     b = simplex[1]
    #
    #     #We just need the edge point, so same point not need to keep duplicate
    #     if a not in EdgePointsIndex:
    #         EdgePointsIndex.append(a)
    #
    #     if b not in EdgePointsIndex:
    #         EdgePointsIndex.append(b)
    #
    #     if c not in EdgePoints2D:
    #         EdgePoints2D.append(c)
    #     if d not in EdgePoints2D:
    #         EdgePoints2D.append(d)

    # print('Edge Points Index\n', EdgePoints)
    return EdgePointsIndex,  EdgePoints2D

def v_sub(a, b):
    return (a[0]-b[0], a[1]-b[1])

def cosine_sign(a, b):
    # determinant
    return a[0]*b[1]-a[1]*b[0]

def get_side(a, b):
    x = cosine_sign(a, b)
    if x < 0:
        return 'LEFT'
    elif x > 0:
        return 'RIGHT'
    elif x == 0:
        return 'RIGHT'
    else:
        return None

def inside_convex_polygon(point, vertices):
    previous_side = None
    n_vertices = len(vertices)

    for n in range(0, n_vertices):
        # if next N will touch the last vertex, then b going back to first vertex
        a, b = vertices[n], vertices[(n+1) % n_vertices]

        affine_segment = v_sub(b, a)
        affine_point = v_sub(point, a)

        current_side = get_side(affine_segment, affine_point)
        # The result should be the same sign, otherwise, it is outside of the convex
        if current_side is None:
            return True  # outside or over an edge
            # return False
        elif previous_side is None:  # first segment
            previous_side = current_side
        elif previous_side != current_side:
            return False

    # if all points don't have False, then the point is inside the convex
    return True

def inside_convex_polygon2(point, vertices):
    region1 = False
    region2 = False
    region3 = False
    region4 = False

    for vertices_no in range(0, len(vertices)):
        myradians = math.atan2(vertices[vertices_no][1] - point[1], vertices[vertices_no][0] - point[0])
        mydegrees = math.degrees(myradians)
        distance = np.sqrt((vertices[vertices_no][1] - point[1])**2 + (vertices[vertices_no][0] - point[0])**2)

        # if distance < 20:
        # First Test
        if 0 <= mydegrees < 90:
            region1 = True
        elif 90 <= mydegrees < 180:
            region2 = True
        elif -180 <= mydegrees < -90:
            region3 = True
        elif -90 <= mydegrees < 0:
            region4 = True



        if region1 and region2 and region3 and region4:
            return True

    return False

# def clockwiseangle_and_distance(point):
#     refvec = [0, 1]
#
#
#     # Vector between point and the origin: v = p - o
#     vector = [point[0]-origin[0], point[1]-origin[1]]
#     # Length of vector: ||v||
#     lenvector = math.hypot(vector[0], vector[1])
#     # If length is zero there is no angle
#     if lenvector == 0:
#         return -math.pi, 0
#     # Normalize vector: v/||v||
#     normalized = [vector[0]/lenvector, vector[1]/lenvector]
#     dotprod  = normalized[0]*refvec[0] + normalized[1]*refvec[1]     # x1*x2 + y1*y2
#     diffprod = refvec[1]*normalized[0] - refvec[0]*normalized[1]     # x1*y2 - y1*x2
#     angle = math.atan2(diffprod, dotprod)
#     # Negative angles represent counter-clockwise angles so we need to subtract them
#     # from 2*pi (360 degrees)
#     if angle < 0:
#         return 2*math.pi+angle, lenvector
#     # I return first the angle because that's the primary sorting criterium
#     # but if two vectors have the same angle then the shorter distance should come first.
#     return angle, lenvector

def ClockWiseSort(pts):
    origin = pts[0]

    def clockwiseangle_and_distance(point):
        refvec = [0, 1]

        # print('point0', point)
        # Vector between point and the origin: v = p - o
        vector = [point[0] - origin[0], point[1] - origin[1]]
        # Length of vector: ||v||
        lenvector = math.hypot(vector[0], vector[1])
        # If length is zero there is no angle
        if lenvector == 0:
            return -math.pi, 0
        # Normalize vector: v/||v||
        normalized = [vector[0] / lenvector, vector[1] / lenvector]
        dotprod = normalized[0] * refvec[0] + normalized[1] * refvec[1]  # x1*x2 + y1*y2
        diffprod = refvec[1] * normalized[0] - refvec[0] * normalized[1]  # x1*y2 - y1*x2
        angle = math.atan2(diffprod, dotprod)
        # Negative angles represent counter-clockwise angles so we need to subtract them
        # from 2*pi (360 degrees)
        if angle < 0:
            return 2 * math.pi + angle, lenvector
        # I return first the angle because that's the primary sorting criterium
        # but if two vectors have the same angle then the shorter distance should come first.
        return angle, lenvector

    pts = sorted(pts, key=clockwiseangle_and_distance)
    return pts

def pnpoly(point, boundary):
    nvert = len(boundary)
    c = False
    for i in range(0, nvert):
        j = i+1
        if j == nvert:
            j = 0

        if ((boundary[i][1]>point[1])!= (boundary[j][1]>point[1])) and (point[0] < ((boundary[j][0]-boundary[i][0])*(point[1]-boundary[i][1])/(boundary[j][1]-boundary[i][1])+boundary[i][0])):

            if c == False:
                c = True
            else:
                c = False

    return c



