import os
import sys

import random
import numpy as np

import math

import schemasim.simulators.physics_simulator as phys_simulator
import schemasim.space.space3D as space

from schemasim.util.geometry import centroid, poseFromTQ, transformVector, fibonacci_sphere, distanceFromInterior, outerAreaFromSurface
from schemasim.util.probability_density import normalizePD, samplePD, uniformQuaternionRPD, uniformBoxRPD

class PhysicsSimulator3D(phys_simulator.PhysicsSimulator):
    def __init__(self):
        super().__init__()
        self._resolutionParticlePD = None
        self._space = space.Space3D()
        return
    def typeName(self):
        return "PhysicsSimulator3D"

    def _getParamset(self, parameter):
        return {"velocity": ["vx", "vy", "vz"], "angular_velocity": ["wx", "wy", "wz"], "translation": ["tx", "ty", "tz"], "rotation": ["rx", "ry", "rz", "rw"]}[parameter]

    def rotationRepresentation(self, obj):
        retq = super().rotationRepresentation(obj)
        return self._space.vectorNormalize(retq)

