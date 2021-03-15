import os
import sys

import random
import numpy as np

import math

import schemasim.simulators.simulator as simulator
import schemasim.simulators.physics_simulator as phys_simulator
import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l2_geometric_primitive_relations as gpr
#import schemasim.schemas.l3_primitive_movement as pm
#import schemasim.schemas.l3_location as location
#import schemasim.schemas.l4_path as path
#import schemasim.schemas.l10_expectation as expectation

import schemasim.objects.example_objects as eo
import schemasim.space.space2D as space

class PhysicsSimulator2D(phys_simulator.PhysicsSimulator):
    def __init__(self, particleSamplingResolution=0.04, translationSamplingResolution=0.1, rotationSamplingResolution=0.1, speedSamplingResolution=0.1, sampleValidationStrictness=0.005, collisionPadding=0.005):
        super().__init__()
        self._resolutionParticlePD = None
        self._space = space.Space2D(particleSamplingResolution=particleSamplingResolution, translationSamplingResolution=translationSamplingResolution, rotationSamplingResolution=rotationSamplingResolution, speedSamplingResolution=speedSamplingResolution, sampleValidationStrictness=sampleValidationStrictness, collisionPadding=collisionPadding)
        return
    def typeName(self):
        return "PhysicsSimulator2D"

    def _getParamset(self, parameter):
        return {"velocity": ["vx", "vy"], "angular_velocity": ["w"], "translation": ["tx", "ty"], "rotation": ["yaw"]}[parameter]

    def rotationRepresentation(self, obj):
        return super().rotationRepresentation(obj)


