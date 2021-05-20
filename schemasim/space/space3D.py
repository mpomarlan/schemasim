import os
import sys

import math
import random

import trimesh

import schemasim.space.space as space

from schemasim.util.geometry import volumeInclusion

from schemasim.util.geometry import centroid, poseFromTQ, scaleMatrix, flipMatrix, transformVector, fibonacci_sphere, outerAreaFromSurface
from schemasim.util.probability_density import normalizePD, samplePD, uniformQuaternionRPD, uniformBoxRPD

class DummyCollisionManager():
    def __init__(self):
        self._objs = []
    def in_collision_single(self, mesh, pose):
        return False
    def add_object(self, name, mesh, pose):
        if name not in self._objs:
            self._objs.append(name)

# A grid of points in 3D space without orientations. Connections between points are bidirectional.
class Grid3D(space.PointGraph):
    def __init__(self, planes=10, lines=10, cols=10, resolution=1, xBack=0, yRight=0, zDown=0, gridQ=(0, 0, 0, 1), validator=None, velocity=1):
        super().__init__()
        self._points = {}
        self._validator = validator
        self._resolution = resolution
        self._planes = planes
        self._lines = lines
        self._cols = cols
        self._xBack = xBack
        self._yRight = yRight
        self._zDown = zDown
        self._velocity = velocity
        self._gridQ = gridQ
        self._iX = transformVector((self._resolution, 0, 0), (0,0,0), self._gridQ)
        self._iY = transformVector((0, self._resolution, 0), (0,0,0), self._gridQ)
        self._iZ = transformVector((0, 0, self._resolution), (0,0,0), self._gridQ)
        if not validator:
            self._validator = space.PointValidator()
        for p in range(self._planes):
            for l in range(self._lines):
                for c in range(self._cols):
                    pointId = (c, l, p)
                    self._points[pointId] = space.GraphPoint(self._validator.isValid(self.pointId2EmbeddingCoordinates(pointId)))
    def pointId2EmbeddingCoordinates(self, pointId):
        return [self._xBack + self._iX[0]*pointId[0] + self._iY[0]*pointId[1] + self._iZ[0]*pointId[2], self._yRight + self._iX[1]*pointId[0] + self._iY[1]*pointId[1] + self._iZ[1]*pointId[2], self._zDown + self._iX[2]*pointId[0] + self._iY[2]*pointId[1] + self._iZ[2]*pointId[2]]
    def embeddingCoordinates2PointId(self, coordinates):
        dx = coordinates[0]-self._xBack
        dy = coordinates[1]-self._yRight
        dz = coordinates[2]-self._zDown
        m = transformVector((dx, dy, dz), (0, 0, 0), (self._gridQ[0], self._gridQ[1], self._gridQ[2], -self._gridQ[3]))
        return (round(m[0]/self._resolution), round(m[1]/self._resolution), round(m[2]/self._resolution))
    def _possibleArrayIncrements(self, pointId):
        pX = [0]
        pY = [0]
        pZ = [0]
        if 0 < pointId[0]:
            pX.append(-1)
        if self._cols-1 > pointId[0]:
            pX.append(1)
        if 0 < pointId[1]:
            pY.append(-1)
        if self._lines-1 > pointId[1]:
            pY.append(1)
        if 0 < pointId[2]:
            pZ.append(-1)
        if self._planes-1 > pointId[2]:
            pZ.append(1)
        return pX, pY, pZ
    def _outgoingNeighbors(self, pointId):
        retq = []
        pX, pY, pZ = self._possibleArrayIncrements(pointId)
        for px in pX:
            for py in pY:
                for pz in pZ:
                    if (0 == px) and (0 == py) and (0 ==pz):
                        continue
                    retq.append((pointId[0] + px, pointId[1] + py, pointId[2] + pz))
        return {x: self.travelTime(pointId, x) for x in retq if ((x in self._points) and self._points[x].valid)}
    def _incomingNeighbors(self, pointId):
        return self._outgoingNeighbors(pointId)
    def _travelTime(self, idA, idB):
        dx = idB[0] - idA[0]
        dy = idB[1] - idA[1]
        dz = idB[2] - idA[2]
        d = math.sqrt(dx*dx + dy*dy + dz*dz)*self._resolution
        return d/self._velocity
    def _graphIngressPoints(self, coordinates):
        retq = {}
        pId = self.embeddingCoordinates2PointId(coordinates)
        for iz in [-1,0,1]:
            for iy in [-1,0,1]:
                for ix in [-1,0,1]:
                    nId = (pId[0]+ix, pId[1]+iy, pId[2]+iz)
                    pS = self.pointId2EmbeddingCoordinates(nId)
                    dx = (pS[0] - coordinates[0])
                    dy = (pS[1] - coordinates[1])
                    dz = (pS[2] - coordinates[2])
                    d = math.sqrt(dx*dx + dy*dy)
                    if (nId in self._points) and (self._points[nId].valid):
                        retq[nId] = d/self._velocity
        return retq
    def _graphEgressPoints(self, coordinates):
        return self._graphIngressPoints(coordinates)

class Space3D(space.Space):
    def __init__(self, particleSamplingResolution=0.04, translationSamplingResolution=0.15, rotationSamplingResolution=0.1, speedSamplingResolution=0.1, sampleValidationStrictness=0.005, collisionPadding=0.01):
        super().__init__(translationSamplingResolution=translationSamplingResolution, rotationSamplingResolution=rotationSamplingResolution, speedSamplingResolution=speedSamplingResolution, sampleValidationStrictness=sampleValidationStrictness, collisionPadding=collisionPadding)
        return
    def makeDefaultSmallTrajector(self):
        r2 = math.sqrt(2)
        s = self._translationSamplingResolution
        return trimesh.Trimesh(vertices=[[s, 0, -s/r2], [-s, 0, -s/r2], [0, s, s/r2], [0, -s, s/r2]],
                       faces=[[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]])
    def loadVolume(self, path, adjustments=None):
        if not path:
            return None
        mesh = trimesh.load(path)
        mesh.process(validate=True)
        if adjustments:
            if (("translation" in adjustments) and (None!=adjustments["translation"])) or (("rotation" in adjustments) and (None!=adjustments["rotation"])):
                translation = self.nullVector()
                rotation = self.identityRotation()
                if ("translation" in adjustments) and (None!=adjustments["translation"]):
                    translation = adjustments["translation"]
                if ("rotation" in adjustments) and (None!=adjustments["rotation"]):
                    rotation = adjustments["rotation"]
                mesh = mesh.apply_transform(poseFromTQ(translation, rotation))
            if ("scale" in adjustments) and (None!=adjustments["scale"]):
                mesh = mesh.apply_transform(scaleMatrix(adjustments["scale"]))
            if ("flip" in adjustments) and (None!=adjustments["flip"]):
                mesh = mesh.apply_transform(flipMatrix(adjustments["flip"]))
        #mesh.export("./stuff.stl")
        return mesh
    def semanticPathModifier(self):
        return ".sem3D"
    def volumePathModifier(self):
        return ".stl"
    def volumeInteriorPathModifier(self):
        return "_interior.stl"
    def dof(self):
        return 3
    def axes(self):
        return [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]
    def origin(self):
        return [0.0, 0.0, 0.0]
    def verticalAxis(self):
        return [0.0, 0.0, 1.0]
    def centroid(self, volume):
        return centroid(volume)
    def verticalComponentIndex(self):
        return 2
    def nullVector(self):
        return [0.0, 0.0, 0.0]
    def identityRotation(self):
        return [0.0,0.0,0.0,1.0]
    def nullVelocity(self):
        return [0.0, 0.0, 0.0]
    def nullAngularVelocity(self):
        return [0.0, 0.0, 0.0]
    def invertTransform(self, transform):
        iq = [transform[1][0], transform[1][1], transform[1][2], -transform[1][3]]
        return [self.transformVector(self.vectorScale(-1, transform[0]), [0,0,0], iq), iq]
    def transformTransform(self, transformA, transformB):
        b1, c1, d1, a1 = transformA[1]
        b2, c2, d2, a2 = transformB[1]
        qNew = [a1*b2+b1*a2+c1*d2-d1*c2, a1*c2-b1*d2+c1*a2+d1*b2, a1*d2+b1*c2-c1*b2+d1*a2, a1*a2-b1*b2-c1*c2-d1*d2]
        return [self.transformVector(transformB[0], transformA[0], transformA[1]), qNew]
    def transformVector(self, vector, translation, orientation):
        return transformVector(vector, translation, orientation)
    def translateVector(self, vector, translation):
        return self.vectorSum(vector, translation)
    def transformVolume(self, volume, translation, rotation):
        return volume.apply_transform(poseFromTQ(translation, rotation))
    def translateVolume(self, volume, translation):
        return volume.apply_translation(translation)
    def poseFromTR(self, translation, rotation):
        return poseFromTQ(translation, rotation)
    def volumeInclusion(self, volumeA, volumeB):
        return volumeInclusion(volumeA, volumeB)
    def distanceFromInterior(self, points, volume):
        if not volume.is_watertight:
            return self.boundaryBoxDiameter(self.volumeBounds(volume))
        if not points:
            return 0.0
        contains = volume.contains(points)
        dists = []
        for c, p in zip(contains, points):
            if c:
                dists.append(0.0)
            else:
                closest, distance, triangle_id = trimesh.proximity.closest_point(volume, [p])
                dists.append(distance)
        return sum(dists)/len(dists)
    def outerAreaFromSurface(self, sa, sb):
        return outerAreaFromSurface(sa, sb, self._translationSamplingResolution*0.1, 2*self._translationSamplingResolution)
    def cubeExtents(self, halfSide):
        if 0.0 > halfSide:
            halfSide = -halfSide
        return [[-halfSide, halfSide], [-halfSide, halfSide], [-halfSide, halfSide]]
    def uniformDirectionSamples(self, samples=100, onlyPositiveQuadrant=False):
        return fibonacci_sphere(samples=samples, only_positive_quadrant=onlyPositiveQuadrant)
    def uniformRotationRPD(self):
        return uniformQuaternionRPD()
    def uniformTranslationBoxRPD(self, extents, translation, rotation, resolution):
        return uniformBoxRPD(extents, translation, rotation, resolution=resolution)
    def volumeBounds(self, volume):
        bbox = list(volume.bounds)
        return [[bbox[0][0], bbox[1][0]], [bbox[0][1], bbox[1][1]], [bbox[0][2], bbox[1][2]]]
    def makeCollisionManager(self):
        retq = None
        try:
            retq = trimesh.collision.CollisionManager()
        except ValueError:
            print("Using dummy collision manager because FCL might not be installed")
            retq = DummyCollisionManager()
        return retq
    def makeRayVolumeIntersector(self, volume):
        return trimesh.ray.ray_triangle.RayMeshIntersector(volume)

