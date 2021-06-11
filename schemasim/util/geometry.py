import math

import numpy as np

import trimesh

def angleDiff(target, current):
    retq = target - current
    if math.pi < retq:
        retq = retq - 2*math.pi
    elif -math.pi > retq:
        retq = retq + 2*math.pi
    return retq

def quaternion2AxisAngle(q):
    angle = 2 * math.acos(q[3])
    n = math.sqrt(1-q[3]*q[3])
    if 0.0001 > n:
        return [0,0,1],0
    axis = [0,0,0]
    axis[0] = q[0] / n
    axis[1] = q[1] / n
    axis[2] = q[2] / n
    return axis, angle

def distanceFromPoint(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    ds = dx*dx + dy*dy + dz*dz
    return math.sqrt(ds)

def distanceFromPoint2D(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    ds = dx*dx + dy*dy
    return math.sqrt(ds)

def sameCell(a, b, resolution=0.05):
    return (distanceFromPoint(a, b) < resolution)

def overlappingCells(a, b,resolution=0.3):
    xOverlap = (a[0] <= b[0] + resolution) and (b[0] - resolution <= a[0])
    yOverlap = (a[1] <= b[1] + resolution) and (b[1] - resolution <= a[1])
    zOverlap = (a[2] <= b[2] + resolution) and (b[2] - resolution <= a[2])
    return xOverlap and yOverlap and zOverlap

def sameCell2D(a, b, resolution=0.05):
    return (distanceFromPoint2D(a, b) < resolution)

def overlappingCells2D(a, b,resolution=0.3):
    xOverlap = (a[0] <= b[0] + resolution) and (b[0] - resolution <= a[0])
    yOverlap = (a[1] <= b[1] + resolution) and (b[1] - resolution <= a[1])
    return xOverlap and yOverlap

def scaleMatrix(scale):
    return [[scale[0], 0, 0, 0], [0, scale[1], 0, 0], [0, 0, scale[2], 0], [0, 0, 0, 1]]

def scaleMatrix2D(scale):
    return [[scale[0], 0, 0], [0, scale[1], 0], [0, 0, 1]]

def flipMatrix(flip):
    fx = 1
    fy = 1
    fz = 1
    if (0 in flip) or ('x' in flip) or ('X' in flip):
        fx = -1
    if (1 in flip) or ('y' in flip) or ('Y' in flip):
        fy = -1
    if (2 in flip) or ('z' in flip) or ('Z' in flip):
        fz = -1
    return [[fx, 0, 0, 0], [0, fy, 0, 0], [0, 0, fz, 0], [0, 0, 0, 1]]

def flipMatrix2D(flip):
    fx = 1
    fy = 1
    if (0 in flip) or ('x' in flip) or ('X' in flip):
        fx = -1
    if (1 in flip) or ('y' in flip) or ('Y' in flip):
        fy = -1
    return [[fx, 0, 0], [0, fy, 0], [0, 0, 1]]

def poseFrom2DTQ(t, q):
    c = math.cos(q[0])
    s = math.sin(q[0])
    pose = [[c,-s,t[0]],[s,c,t[1]],[0,0,1]]
    pose = np.array(pose)
    return pose

def poseFromTQ(t, q):
    pose = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    t2 = w*x
    t3 = w*y
    t4 = w*z
    t5 = -x*x
    t6 = x*y
    t7 = x*z
    t8 = -y*y
    t9 = y*z
    t10 = -z*z
    pose[0][0] = 2*(t8 + t10 + 0.5)
    pose[0][1] = 2*(t6 - t4)
    pose[0][2] = 2*(t3 + t7)
    pose[1][0] = 2*(t4 + t6)
    pose[1][1] = 2*(t5 + t10 + 0.5)
    pose[1][2] = 2*(t9 - t2)
    pose[2][0] = 2*(t7 - t3)
    pose[2][1] = 2*(t2 + t9)
    pose[2][2] = 2*(t5 + t8 + 0.5)
    pose[0][3] = t[0]
    pose[1][3] = t[1]
    pose[2][3] = t[2]
    pose = np.array(pose)
    return pose

def transformVector(v, t, q):
    v = list(v)
    pose = poseFromTQ(t, q)
    rv = [0,0,0,0]
    nv = v + [1.0]
    for k in list(range(4)):
        for j in list(range(4)):
            rv[k] = rv[k] + pose[k][j]*nv[j]
    return rv[:3]

def transform2DVector(v, t, q):
    v = list(v)
    pose = poseFrom2DTQ(t, q)
    rv = [0,0,0]
    nv = v + [1.0]
    for k in list(range(3)):
        for j in list(range(3)):
            rv[k] = rv[k] + pose[k][j]*nv[j]
    return rv[:2]

def centroid(points):
    centroid = [0.0,0.0,0.0]
    for p in points:
        centroid = [centroid[0] + p[0], centroid[1] + p[1], centroid[2] + p[2]]
    n = len(points)
    centroid = [centroid[0]/n, centroid[1]/n, centroid[2]/n]
    return centroid

def fibonacci_sphere(samples=1, only_positive_quadrant=True):
    points = []
    offset = 2./samples
    increment = math.pi * (3. - math.sqrt(5.));
    for i in range(samples):
        y = ((i * offset) - 1) + (offset / 2);
        r = math.sqrt(1 - pow(y,2))
        phi = (i % samples) * increment
        x = math.cos(phi) * r
        z = math.sin(phi) * r
        if (not only_positive_quadrant) or ((0.0 <= x) and (0.0 <= y) and  (0.0 <= z)):
            points.append([x,y,z])
    return points

def outerAreaFromSurface(movingSurface, targetSurface, resolution=0.05, samplingResolution=0.3):
    nonOverlapping = 0
    for a in movingSurface:
        overlaps = 0
        for b in targetSurface:
            if sameCell(a, b, resolution):
               overlaps = 4
               break
            if overlappingCells(a, b, samplingResolution):
               overlaps = overlaps + 1
        if overlaps < 4:
            nonOverlapping = nonOverlapping + 1
    if 0 == nonOverlapping:
        return 0.0
    normedArea = ((1.0*nonOverlapping)/len(movingSurface))
    if distanceFromPoint(centroid(movingSurface), centroid(targetSurface)) <= samplingResolution:
        normedArea = normedArea*0.01
    return normedArea*0.5

def outerAreaFromSurface2D(movingSurface, targetSurface, resolution=0.05, samplingResolution=0.3):
    nonOverlapping = 0
    for a in movingSurface:
        overlaps = 0
        for b in targetSurface:
            if sameCell2D(a, b, resolution):
               overlaps = 2
               break
            if overlappingCells2D(a, b, samplingResolution):
               overlaps = overlaps + 1
        if overlaps < 2:
            nonOverlapping = nonOverlapping + 1
    if 0 == nonOverlapping:
        return 0.0
    normedArea = ((1.0*nonOverlapping)/len(movingSurface))
    if distanceFromPoint(centroid(movingSurface), centroid(targetSurface)) <= samplingResolution:
        normedArea = normedArea*0.01
    return normedArea*0.5

