import os
import sys

import math
import numpy as np

import itertools

class Space:
    def __init__(self, particleSamplingResolution=0.04, translationSamplingResolution=0.1, rotationSamplingResolution=0.1, speedSamplingResolution=0.1, sampleValidationStrictness=0.005, collisionPadding=0.005):
        super().__init__()
        self._particleSamplingResolution = particleSamplingResolution
        self._translationSamplingResolution = translationSamplingResolution
        self._rotationSamplingResolution = rotationSamplingResolution
        self._speedSamplingResolution = speedSamplingResolution
        self._sampleValidationStrictness = sampleValidationStrictness
        self._collisionPadding = collisionPadding
        return
    def loadVolume(self, path):
        return None
    def collisionPadding(self):
        return self._collisionPadding
    def particleSamplingResolution(self):
        return self._particleSamplingResolution
    def translationSamplingResolution(self):
        return self._translationSamplingResolution
    def rotationSamplingResolution(self):
        return self._rotationSamplingResolution
    def sampleValidationStrictness(self):
        return self._sampleValidationStrictness
    def speedSamplingResolution(self):
        return self._speedSamplingResolution

    def semanticPathModifier(self):
        return None
    def volumePathModifier(self):
        return None
    def volumeInteriorPathModifier(self):
        return None
    def dof(self):
        return None
    def axes(self):
        return None
    def origin(self):
        return None
    def verticalAxis(self):
        return None
    def centroid(self, volume):
        return None
    def verticalComponentIndex(self):
        return None
    def vectorAboveOrigin(self, x):
        retq = self.origin()
        retq[self.verticalComponentIndex()] = x
        return retq
    def floorLevel(self, volume):
        if not volume:
            return 0.0
        return self.volumeBounds(volume)[self.verticalComponentIndex()][0]
    def nullVector(self):
        return None
    def identityRotation(self):
        return None
    def nullVelocity(self):
        return None
    def nullAngularVelocity(self):
        return None
    def verticalVectorComponent(self, v):
        return v[self.verticalComponentIndex()]
    def vectorNorm(self, v):
        return math.sqrt(self.vectorDotProduct(v, v))
    def vectorNormalize(self, v):
        return self.vectorScale(1.0/self.vectorNorm(v), v)
    def vectorDifference(self, va, vb):
        return self.vectorSum(va, self.vectorScale(-1.0, vb))
    def vectorSum(self, va, vb):
        if len(va) != len(vb):
            return None
        return [x + y for (x, y) in zip(va, vb)]
    def vectorScale(self, scalar, vector):
        return [scalar*x for x in vector]
    def vectorDotProduct(self, va, vb):
        if len(va) != len(vb):
            return None
        retq = 0.0
        for (x, y) in zip(va, vb):
            retq = retq + x*y
        return retq
    def transformVector(self, vector, translation, orientation):
        return None
    def translateVector(self, vector, translation):
        return None
    def transformVolume(self, volume, translation, rotation):
        return None
    def translateVolume(self, volume, translation):
        return None
    def poseFromTR(self, translation, rotation):
        return None
    def volumeInclusion(self, volumeA, volumeB):
        return None
    def distanceFromInterior(self, point, volume, volumeRayIntersector):
        return None
    def outerAreaFromSurface(self, sa, sb):
        return None
    def cubeExtents(self, halfSide):
        return None
    def uniformDirectionSamples(self, samples=0, onlyPositiveQuadrant=False):
        return None
    def uniformRotationRPD(self):
        return None
    def uniformTranslationBoxRPD(self, extents, translation, rotation, resolution):
        return None
    def pointCloudBounds(self, points):
        if not points:
            return None
        mins = list(points[0])
        maxs = list(points[0])
        for p in points[1:]:
            if len(p) != len(mins):
                return None
            for k in list(range(len(mins))):
                if p[k] < mins[k]:
                    mins[k] = p[k]
                if maxs[k] < p[k]:
                    maxs[k] = p[k]
        return [[x, y] for (x, y) in zip(mins, maxs)]
    def volumeBounds(self, volume):
        return None
    def boundaryBoxCorners(self, bbox):
        return list(itertools.product(*bbox))
    def boundaryBoxDiameter(self, bbox):
        minP = [x[0] for x in bbox]
        maxP = [x[1] for x in bbox]
        return self.vectorNorm(self.vectorDifference(minP, maxP))
    def orthogonalBasis(self, vector):
        dots = [(math.fabs(self.vectorDotProduct(vector, x)), x) for x in self.axes()]
        axes = [x[1] for x in sorted(dots, key=lambda y: y[0])[:-1]]
        rawbasis = [vector] + [self.vectorNormalize(self.vectorSum(vector, x)) for x in axes]
        basis = []
        for k in list(range(1, len(rawbasis))):
            v = rawbasis[k]
            for j in list(range(k)):
                v = self.vectorDifference(v, self.vectorScale(self.vectorDotProduct(v, rawbasis[j]), rawbasis[j]))
            basis.append(self.vectorNormalize(v))
        return basis
    def planePoints(self, rayDir, refP, diameter):
        basis = self.orthogonalBasis(rayDir)
        steps = []
        for c in refP[1:]:
            steps.append(np.arange(-diameter, diameter, self._translationSamplingResolution).tolist())
        opoints = list(itertools.product(*steps))
        retq = []
        for opoint in opoints:
            disp = self.origin()
            for (s, v) in zip(opoint, basis):
                disp = self.vectorSum(disp, self.vectorScale(s, v))
            retq.append(self.vectorSum(refP, disp))
        return retq
    def projectRaysOnVolume(self, volume, rayDir, rayOffset):
        volumeRayIntersector = self.makeRayVolumeIntersector(volume)
        bbox = self.volumeBounds(volume)
        corners = self.boundaryBoxCorners(bbox)
        diameter = self.boundaryBoxDiameter(bbox) + self._translationSamplingResolution
        refP = corners[0]
        dotP = self.vectorDotProduct(rayDir, refP)
        for c in corners[1:]:
            dotNP = self.vectorDotProduct(rayDir, c)
            if dotNP < dotP:
                dotP = dotNP
                refP = c
        refP = self.vectorSum(rayOffset, refP)
        ray_dirs = []
        ray_centers = self.planePoints(rayDir, refP, diameter)
        r2c = {}
        k = 0
        for rc in ray_centers:
            ray_dirs.append(rayDir)
            r2c[k] = rc
            k = k + 1
        triangs, rays = volumeRayIntersector.intersects_id(ray_centers, ray_dirs, multiple_hits=False)
        rays = sorted(list(set(list(rays))))
        retq = []
        for r in rays:
            retq.append(self.vectorSum(self.vectorDifference(r2c[r], rayOffset), self.vectorScale(-self._collisionPadding, rayDir)))
        return retq
    def makeCollisionManager(self):
        return None
    def makeRayVolumeIntersector(self, volume):
        return None

