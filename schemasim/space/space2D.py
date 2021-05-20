import os
import sys

import math
import random

import trimesh

import schemasim.space.space as space

from schemasim.util.geometry import volumeInclusion

from schemasim.util.geometry import centroid, poseFrom2DTQ, scaleMatrix2D, flipMatrix2D, transform2DVector, outerAreaFromSurface2D, angleDiff
from schemasim.util.probability_density import normalizePD, samplePD, uniformBox2DRPD

# A grid of points in 2D space with orientations. The orientation of a point can take one of eight values.
# Neighborhood relations are constrained s.t. they represent simple trajectories of a "car" between close points.
class Grid2DVW8(space.PointGraph):
    def __init__(self, lines=10, cols=10, resolution=1, xLeft=0, yDown=0, gridYaw=0, validator=None, velocity=1, angularVelocity=1):
        super().__init__()
        self._points = {}
        self._validator = validator
        self._resolution = resolution
        self._lines = lines
        self._cols = cols
        self._xLeft = xLeft
        self._yDown = yDown
        self._velocity = velocity
        self._angularVelocity = angularVelocity
        self._gridYaw = gridYaw
        self._c = math.cos(gridYaw)
        self._s = math.sin(gridYaw)
        self._iX = [resolution*self._c, resolution*self._s]
        self._iY = [-resolution*self._s, resolution*self._c]
        self._iYaw = math.pi/4
        if not validator:
            self._validator = space.PointValidator()
        for l in range(self._lines):
            for c in range(self._cols):
                isValid = self._validator.isValid(self.pointId2EmbeddingCoordinates((c, l, 0)))
                for yaw in range(8):
                    pointId = (c, l, yaw)
                    self._points[pointId] = space.GraphPoint(isValid)
    def __str__(self):
        retq = ""
        for l in reversed(range(self._lines)):
            for c in range(self._cols):
                ch = "."
                if not self._points[(c, l, 0)].valid:
                    ch = "#"
                retq = retq + ch
            retq = retq+"\n"
        return retq
    def pointId2EmbeddingCoordinates(self, pointId):
        return [self._xLeft + self._iX[0]*pointId[0] + self._iY[0]*pointId[1], self._yDown + self._iX[1]*pointId[0] + self._iY[1]*pointId[1], self._gridYaw + self._iYaw*pointId[2]]
    def embeddingCoordinates2PointId(self, coordinates):
        dx = coordinates[0]-self._xLeft
        dy = coordinates[1]-self._yDown
        mx = self._c*dx + self._s*dy
        my = -self._s*dx + self._c*dy
        return (round(mx/self._resolution), round(my/self._resolution), round((coordinates[2]-self._gridYaw)/self._iYaw)%8)
    def _possibleArrayIncrements(self, pointId):
        pX = [0]
        pY = [0]
        if 0 < pointId[0]:
            pX.append(-1)
        if self._cols-1 > pointId[0]:
            pX.append(1)
        if 0 < pointId[1]:
            pY.append(-1)
        if self._lines-1 > pointId[1]:
            pY.append(1)
        return pX, pY
    def _outgoingNeighbors(self, pointId):
        retq = [(pointId[0], pointId[1], (pointId[2]+1)%8), (pointId[0], pointId[1], (pointId[2]-1)%8)]
        aX = [1, 1, 0, -1, -1, -1, 0, 1]
        aY = [0, 1, 1, 1, 0, -1, -1, -1]
        pX, pY = self._possibleArrayIncrements(pointId)
        if (aX[pointId[2]] in pX) and (aY[pointId[2]] in pY):
            retq.append((pointId[0] + aX[pointId[2]], pointId[1] + aY[pointId[2]], pointId[2]))
        return {x: self.travelTime(pointId, x) for x in retq if ((x in self._points) and self._points[x].valid)}
    def _incomingNeighbors(self, pointId):
        retq = [(pointId[0], pointId[1], (pointId[2]+1)%8), (pointId[0], pointId[1], (pointId[2]-1)%8)]
        aX = [-1, -1, 0, 1, 1, 1, 0, -1]
        aY = [0, -1, -1, -1, 0, 1, 1, 1]
        pX, pY = self._possibleArrayIncrements(pointId)
        if (aX[pointId[2]] in pX) and (aY[pointId[2]] in pY):
            retq.append((pointId[0] + aX[pointId[2]], pointId[1] + aY[pointId[2]], pointId[2]))
        return {x: self.travelTime(x, pointId) for x in retq if ((x in self._points) and self._points[x].valid)}
    def _travelTime(self, idA, idB):
        if (idA[0] == idB[0]) and (idA[1] == idB[1]):
            return abs(angleDiff(idA[2], idB[2]))/self._angularVelocity
        dx = idB[0] - idA[0]
        dy = idB[1] - idA[1]
        d = math.sqrt(dx*dx + dy*dy)*self._resolution
        tYaw = math.atan2(dy, dx)
        return (d/self._velocity) + ((abs(angleDiff(tYaw, idA[2])) + abs(angleDiff(idB[2], tYaw)))/self._angularVelocity)
    def _graphIngressPoints(self, coordinates):
        retq = {}
        pId = self.embeddingCoordinates2PointId(coordinates)
        for iy in [-1,0,1]:
            for ix in [-1,0,1]:
                nId = (pId[0]+ix, pId[1]+iy, pId[2])
                pS = self.pointId2EmbeddingCoordinates(nId)
                dx = (pS[0] - coordinates[0])
                dy = (pS[1] - coordinates[1])
                d = math.sqrt(dx*dx + dy*dy)
                if 0.01*self._resolution > d:
                    if (nId in self._points) and (self._points[nId].valid):
                        return {nId: abs(angleDiff(pS[2], coordinates[2]))/self._angularVelocity}
                else:
                    tYaw = math.atan2(dy, dx)
                    nId = self.embeddingCoordinates2PointId((pS[0], pS[1], tYaw))
                    if (nId in self._points) and (self._points[nId].valid):
                        pS = self.pointId2EmbeddingCoordinates(nId)
                        retq[nId] = d/self._velocity + (abs(angleDiff(tYaw, coordinates[2])) + abs(angleDiff(pS[2], tYaw)))/self._angularVelocity
        return retq
    def _graphEgressPoints(self, coordinates):
        retq = {}
        pId = self.embeddingCoordinates2PointId(coordinates)
        for iy in [-1,0,1]:
            for ix in [-1,0,1]:
                nId = (pId[0]+ix, pId[1]+iy, pId[2])
                pS = self.pointId2EmbeddingCoordinates(nId)
                dx = -(pS[0] - coordinates[0])
                dy = -(pS[1] - coordinates[1])
                d = math.sqrt(dx*dx + dy*dy)
                if 0.01*self._resolution > d:
                    if (nId in self._points) and (self._points[nId].valid):
                        return {nId: abs(angleDiff(coordinates[2], pS[2]))/self._angularVelocity}
                else:
                    tYaw = math.atan2(dy, dx)
                    nId = self.embeddingCoordinates2PointId((pS[0], pS[1], tYaw))
                    if (nId in self._points) and (self._points[nId].valid):
                        pS = self.pointId2EmbeddingCoordinates(nId)
                        retq[nId] = d/self._velocity + (abs(angleDiff(coordinates[2], tYaw)) + abs(angleDiff(tYaw, pS[2])))/self._angularVelocity
        return retq

class Shape2D:
    def __init__(self, vertices=[], faces=[]):
        self.vertices = vertices
        self.faces = faces
        self.bounds = [[None, None], [None, None]]
        for v in self.vertices:
            if (None == self.bounds[0][0]) or (self.bounds[0][0] > v[0]):
                self.bounds[0][0] = v[0]
            if (None == self.bounds[0][1]) or (self.bounds[0][1] > v[1]):
                self.bounds[0][1] = v[1]
            if (None == self.bounds[1][0]) or (self.bounds[1][0] < v[0]):
                self.bounds[1][0] = v[0]
            if (None == self.bounds[1][1]) or (self.bounds[1][1] < v[1]):
                self.bounds[1][1] = v[1]
    def apply_translation(self, translation):
        vertices = []
        for v in self.vertices:
            vertices.append([v[0] + translation[0], v[1] + translation[1]])
        return Shape2D(vertices=vertices, faces=self.faces)
    def apply_transform(self, transform):
        vertices = []
        for v in self.vertices:
            vertices.append([v[0]*transform[0][0] + v[1]*transform[0][1] + transform[0][2], v[0]*transform[1][0] + v[1]*transform[1][1] + transform[1][2]])
        return Shape2D(vertices=vertices, faces=self.faces)

class Space2D(space.Space):
    def __init__(self, particleSamplingResolution=0.04, translationSamplingResolution=0.15, rotationSamplingResolution=0.1, speedSamplingResolution=0.1, sampleValidationStrictness=0.005, collisionPadding=0.01):
        super().__init__(translationSamplingResolution=translationSamplingResolution, rotationSamplingResolution=rotationSamplingResolution, speedSamplingResolution=speedSamplingResolution, sampleValidationStrictness=sampleValidationStrictness, collisionPadding=collisionPadding)
        return
    def makeDefaultSmallTrajector(self):
        r3 = math.sqrt(3)
        s = self._translationSamplingResolution
        # TODO: establish a data structure to hold 2D data
        return Shape2D(vertices=[[s/r3, 0], [-s/(2*r3), s/2], [-s/(2*r3), -s/2]],
                       faces=[[0, 1], [1,2], [2, 0]])
    def loadVolume(self, path, adjustments=None):
        # TODO: establish a data structure to hold 2D data
        if not path:
            return None
        vertices = []
        faces = []
        lines = [x for x in open(path).read().splitlines() if x]
        sl = lines[0].split()
        vertCount = int(sl[0])
        for l in lines[1:]:
            sl = l.split()
            if 0 < vertCount:
                vertices.append([float(sl[0]), float(sl[1])])
                vertCount = vertCount - 1
            else:
                faces.append([int(sl[0]), int(sl[1])])
        mesh = Shape2D(vertices=vertices, faces=faces)
        if adjustments:
            if (("translation" in adjustments) and (None!=adjustments["translation"])) or (("rotation" in adjustments) and (None!=adjustments["rotation"])):
                translation = self.nullVector()
                rotation = self.identityRotation()
                if ("translation" in adjustments) and (None!=adjustments["translation"]):
                    translation = adjustments["translation"]
                if ("rotation" in adjustments) and (None!=adjustments["rotation"]):
                    rotation = adjustments["rotation"]
                mesh = mesh.apply_transform(poseFrom2DTQ(translation, rotation))
            if ("scale" in adjustments) and (None!=adjustments["scale"]):
                mesh = mesh.apply_transform(scaleMatrix2D(adjustments["scale"]))
            if ("flip" in adjustments) and (None!=adjustments["flip"]):
                mesh = mesh.apply_transform(flipMatrix2D(adjustments["flip"]))
        #mesh.export("./stuff.stl")
        return mesh
    def semanticPathModifier(self):
        return ".sem2D"
    def volumePathModifier(self):
        # TODO: establish a data structure to hold 2D data
        return ".2d"
    def volumeInteriorPathModifier(self):
        # TODO: establish a data structure to hold 2D data
        return "_interior.2d"
    def dof(self):
        return 2
    def axes(self):
        return [1.0, 0.0], [0.0, 1.0]
    def origin(self):
        return [0.0, 0.0]
    def verticalAxis(self):
        return [0.0, 1.0]
    def centroid(self, volume):
        # TODO: establish a data structure to hold 2D data
        retq = [0, 0]
        for v in volume.vertices:
            retq = self.vectorSum(v, retq)
        retq = self.vectorScale(1.0/len(volume.vertices), retq)
        return retq
    def verticalComponentIndex(self):
        return 1
    def nullVector(self):
        return [0.0, 0.0]
    def identityRotation(self):
        return [0.0]
    def nullVelocity(self):
        return [0.0, 0.0]
    def nullAngularVelocity(self):
        return [0.0]
    def invertTransform(self, transform):
        alpha = -transform[1][0]
        v = self.transformVector(transform[0], [0,0], [alpha])
        return [[-v[0], -v[1]], [alpha]]
    def transformTransform(self, transformA, transformB):
        alpha = transformA[1][0] + transformB[1][0]
        if math.pi < alpha:
            alpha = alpha - 2*math.pi
        elif -math.pi > alpha:
            alpha = alpha + 2*math.pi
        return [self.transformVector(transformB[0], transformA[0], transformA[1]), [alpha]]
    def transformVector(self, vector, translation, orientation):
        return transform2DVector(vector, translation, orientation)
    def translateVector(self, vector, translation):
        return self.vectorSum(vector, translation)
    def transformVolume(self, volume, translation, rotation):
        # TODO: establish a data structure to hold 2D data
        return volume.apply_transform(poseFrom2DTQ(translation, rotation))
    def translateVolume(self, volume, translation):
        # TODO: establish a data structure to hold 2D data
        return volume.apply_translation(translation)
    def poseFromTR(self, translation, rotation):
        # TODO: establish a data structure to hold 2D data
        return poseFromTQ(translation, rotation)
    def volumeInclusion(self, volumeA, volumeB):
        # TODO
        return volumeInclusion(volumeA, volumeB)
    def distanceFromInterior(self, points, volume):
        # TODO
        return 0.0
    def outerAreaFromSurface(self, sa, sb):
        return outerAreaFromSurface2D(sa, sb, self._translationSamplingResolution*0.1, 2*self._translationSamplingResolution)
    def cubeExtents(self, halfSide):
        if 0.0 > halfSide:
            halfSide = -halfSide
        return [[-halfSide, halfSide], [-halfSide, halfSide]]
    def uniformDirectionSamples(self, samples=100, onlyPositiveQuadrant=False):
        retq = []
        q = 0
        a = (2*math.pi)
        if onlyPositiveQuadrant:
            a = math.pi/2.0
        dq = a/samples
        for k in range(samples):
            retq.append([math.cos(q), math.sin(q)])
            q = q + dq
        return retq
    def uniformRotationRPD(self):
        rpd = []
        N = 100
        q = -math.pi
        dq = (2*math.pi)/N
        for k in range(N):
            rpd.append([1.0, [q]])
            q = q + dq
        return rpd
    def uniformTranslationBoxRPD(self, extents, translation, rotation, resolution):
        return uniformBox2DRPD(extents, translation, rotation, resolution=resolution)
    def volumeBounds(self, volume):
        bbox = list(volume.bounds)
        return [[bbox[0][0], bbox[1][0]], [bbox[0][1], bbox[1][1]]]
    def makeCollisionManager(self):
        # TODO
        retq = None
        try:
            retq = trimesh.collision.CollisionManager()
        except ValueError:
            print("Using dummy collision manager because FCL might not be installed")
            retq = DummyCollisionManager()
        return retq
    def makeRayVolumeIntersector(self, volume):
        # TODO
        return trimesh.ray.ray_triangle.RayMeshIntersector(volume)

