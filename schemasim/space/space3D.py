import os
import sys

import math

import trimesh

import schemasim.space.space as space

from schemasim.util.geometry import volumeInclusion

from schemasim.util.geometry import centroid, poseFromTQ, transformVector, fibonacci_sphere, distanceFromInterior, outerAreaFromSurface
from schemasim.util.probability_density import normalizePD, samplePD, uniformQuaternionRPD, uniformBoxRPD

class Space3D(space.Space):
    def __init__(self, particleSamplingResolution=0.04, translationSamplingResolution=0.15, rotationSamplingResolution=0.1, speedSamplingResolution=0.1, sampleValidationStrictness=0.005, collisionPadding=0.01):
        super().__init__(translationSamplingResolution=translationSamplingResolution, rotationSamplingResolution=rotationSamplingResolution, speedSamplingResolution=speedSamplingResolution, sampleValidationStrictness=sampleValidationStrictness, collisionPadding=collisionPadding)
        return
    def loadVolume(self, path):
        if not path:
            return None
        return trimesh.load(path)
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
    def distanceFromInterior(self, point, volume, volumeRayIntersector):
        return 2.0*distanceFromInterior(point, volume, volumeRayIntersector)/self.boundaryBoxDiameter(self.volumeBounds(volume))
    def outerAreaFromSurface(self, sa, sb):
        return outerAreaFromSurface(sa, sb)
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
        return trimesh.collision.CollisionManager()
    def makeRayVolumeIntersector(self, volume):
        return trimesh.ray.ray_triangle.RayMeshIntersector(volume)
