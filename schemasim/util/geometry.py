import math

import numpy as np

import trimesh

def distanceFromPoint(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    ds = dx*dx + dy*dy + dz*dz
    return math.sqrt(ds)

def sameCell(a, b):
    return (distanceFromPoint(a, b) < 0.05)

def overlappingCells(a, b):
    xOverlap = (a[0] <= b[0] + 0.3) and (b[0] - 0.3 <= a[0])
    yOverlap = (a[1] <= b[1] + 0.3) and (b[1] - 0.3 <= a[1])
    zOverlap = (a[2] <= b[2] + 0.3) and (b[2] - 0.3 <= a[2])
    return xOverlap and yOverlap and zOverlap

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
    pose = poseFromTQ(t, q)
    rv = [0,0,0,0]
    nv = v + [1.0]
    for k in list(range(4)):
        for j in list(range(4)):
            rv[k] = rv[k] + pose[k][j]*nv[j]
    return rv[:3]

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

def distanceFromInterior(point, volume, volumeRayIntersector):
    triangs, rays = volumeRayIntersector.intersects_id([point], [[0,0,1]], multiple_hits=True)
    if 1 == len(triangs) % 2:
        # point is inside
        return 0
    closest, distance, triangle_id = trimesh.proximity.closest_point(volume, [point])
    return distance[0]

def volumeInclusion(movingVolume, targetVolume):
    targetVolumeIntersector = trimesh.ray.ray_triangle.RayMeshIntersector(targetVolume)
    rayDirs = []
    intersections = []
    for p in movingVolume.vertices:
        rayDirs.append([0.0,0.0,1.0])
        intersections.append(0)
    triangs, rays = targetVolumeIntersector.intersects_id(list(movingVolume.vertices), rayDirs, multiple_hits=True)
    for r in rays:
        intersections[r] = intersections[r] + 1
    nonIns = 0
    for i in intersections:
        if 0 == (i%2):
            nonIns = nonIns + 1
    if len(movingVolume.vertices) < nonIns*6:
        return False
    return True

def outerAreaFromSurface(movingSurface, targetSurface):
    nonOverlapping = 0
    for a in movingSurface:
        overlaps = 0
        for b in targetSurface:
            if sameCell(a, b):
               overlaps = 4
               break
            if overlappingCells(a, b):
               overlaps = overlaps + 1
        if overlaps < 4:
            nonOverlapping = nonOverlapping + 1
    if 0 == nonOverlapping:
        return 0.0
    normedArea = ((1.0*nonOverlapping)/len(movingSurface))
    if distanceFromPoint(centroid(movingSurface), centroid(targetSurface)) <= 0.2:
        normedArea = normedArea*0.01
    return normedArea*0.5

