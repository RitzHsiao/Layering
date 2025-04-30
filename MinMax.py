import numpy as np

def MinMax(Min, Max, numb):
    if numb < Min:
        Min = numb
    if numb > Max:
        Max = numb
    return Min, Max

def TopBotZ (point):
    for point_no in range(0, len(point)):
        if point_no == 0:
            MaxZpoint = point[point_no][2]
            MinZpoint = point[point_no][2]

        else:
            if point[point_no][2] > MaxZpoint:
                MaxZpoint = point[point_no][2]

            if point[point_no][2] < MinZpoint:
                MinZpoint = point[point_no][2]


    return MaxZpoint, MinZpoint

def CylinderErase(pt1, pt2, r, q):
    # If the point is inside, then it will be True
    Point_is_inside = False

    vec = np.subtract(pt2, pt1)
    const = r*np.linalg.norm(vec)

    top = np.subtract(q, pt1)       # Raw Point is at the top of LastPoint
    down = np.subtract(q, pt2)      # Raw Point is at the bottom of NextPoint

    if np.linalg.norm(np.cross(top, vec)) <= const:
        if np.dot(top, vec) >= 0 and np.dot(down, vec)<=0:
            Point_is_inside = True

    return Point_is_inside

def PointFilter(point, pointidx, normal, x1, x2, z1, z2):
    # We just deal with point within range of x1 to x2
    Max, Min = TopBotZ(point)

    if x2 < x1:
        temp = x1
        x1 = x2
        x2 = temp

    r = abs(x1 - x2) / 2
    MaxPoint = [x1 + r, x1 + r, Max]
    MinPoint = [x1 + r, x1 + r, Min]

    NewPoint = []
    NewNormal = []
    NewPointIdx = []

    for point_no in range(0, len(point)):
        if point[point_no][2] > z1 and point[point_no][2] < z2:
            if not CylinderErase(MaxPoint, MinPoint, r, point[point_no]):
                NewPoint.append(point[point_no])
                NewNormal.append(normal[point_no])
                NewPointIdx.append(pointidx[point_no])

    return NewPoint, NewPointIdx, NewNormal
