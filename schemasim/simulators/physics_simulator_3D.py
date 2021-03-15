import os
import sys

import random
import numpy as np

import math

import schemasim.simulators.physics_simulator as phys_simulator
import schemasim.space.space3D as space

class PhysicsSimulator3D(phys_simulator.PhysicsSimulator):
    def __init__(self, particleSamplingResolution=0.04, translationSamplingResolution=0.1, rotationSamplingResolution=0.1, speedSamplingResolution=0.1, sampleValidationStrictness=0.005, collisionPadding=0.005):
        super().__init__()
        self._resolutionParticlePD = None
        self._space = space.Space3D(particleSamplingResolution=particleSamplingResolution, translationSamplingResolution=translationSamplingResolution, rotationSamplingResolution=rotationSamplingResolution, speedSamplingResolution=speedSamplingResolution, sampleValidationStrictness=sampleValidationStrictness, collisionPadding=collisionPadding)
        return
    def typeName(self):
        return "PhysicsSimulator3D"

    def _getParamset(self, parameter):
        return {"velocity": ["vx", "vy", "vz"], "angular_velocity": ["wx", "wy", "wz"], "translation": ["tx", "ty", "tz"], "rotation": ["rx", "ry", "rz", "rw"]}[parameter]

    def rotationRepresentation(self, obj):
        retq = super().rotationRepresentation(obj)
        return self._space.vectorNormalize(retq)

