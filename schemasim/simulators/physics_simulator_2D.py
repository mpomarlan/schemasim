import os
import sys

import random
import numpy as np

import math

import schemasim.simulators.simulator as simulator
import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l2_geometric_primitive_relations as gpr
#import schemasim.schemas.l3_primitive_movement as pm
#import schemasim.schemas.l3_location as location
#import schemasim.schemas.l4_path as path
#import schemasim.schemas.l10_expectation as expectation

import schemasim.objects.example_objects as eo
import schemasim.space.space2D as space

from schemasim.util.geometry import centroid, poseFromTQ, transformVector, fibonacci_sphere, distanceFromInterior, outerAreaFromSurface
from schemasim.util.probability_density import normalizePD, samplePD, uniformQuaternionRPD, uniformBoxRPD

class PhysicsSimulator2D(simulator.PhysicsSimulator):
    def __init__(self):
        super().__init__()
        self._resolutionParticlePD = None
        self._space = space.Space2D()
        return
    def typeName(self):
        return "PhysicsSimulator2D"
