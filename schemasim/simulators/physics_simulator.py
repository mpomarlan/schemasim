import os
import sys

import random
import numpy as np

import math

import schemasim.simulators.simulator as simulator

class PhysicsSimulator(simulator.Simulator):
    # This class should not be used directly; it is a base for PhysicsSimulator3D, PhysicsSimulator2D, which themselves are bases for classes
    # interfacing with actual simulators
    def __init__(self):
        super().__init__()
        self._densityGenerationFns["mass"] = self._getMassPD
        self._densityGenerationFns["restitution"] = self._getRestitutionPD
        self._densityGenerationFns["friction"] = self._getFrictionPD
        self._densityGenerationFns["linear_damping"] = self._getLinDmpPD
        self._densityGenerationFns["angular_damping"] = self._getAngDmpPD
        self._assignmentFns["mass"] = self._assignMass
        self._assignmentFns["restitution"] = self._assignRestitution
        self._assignmentFns["friction"] = self._assignFriction
        self._assignmentFns["linear_damping"] = self._assignLinDamp
        self._assignmentFns["angular_damping"] = self._assignAngDamp
        return
    def _initializeConstraintList(self):
        retq = super()._initializeConstraintList()
        retq.update({"mass": [], "restitution": [], "friction": [], "linear_damping": [], "angular_damping": []})
        return retq

    def _isIndividual(self, obj):
        return obj._parameters["physics_type"] in ["rigid_body", "soft_body"]
    def _isParticulate(self, obj):
        return obj._parameters["physics_type"] in ["impulse_particles"]

    def space(self):
        return self._space
    def typeName(self):
        return "PhysicsSimulator"
    def compatiblePhysics(self, schemas):
        for s in schemas:
            if "ParameterizedSchema" in s._meta_type:
                if ("physics_type" not in s._parameters) or (s._parameters["physics_type"] not in ["rigid_body", "soft_body", "impulse_particles"]):
                    return False
        return True

    def _assignMass(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "mass")
    def _assignRestitution(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "restitution")
    def _assignFriction(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "friction")
    def _assignLinDamp(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "linear_damping")
    def _assignAngDamp(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "angular_damping")

    def _getPhysicsTypePD(self, constraints):
        return [[1.0, "rigid_body"]]
    def _getMassPD(self, constraints):
        return None
    def _getRestitutionPD(self, constraints):
        return None
    def _getFrictionPD(self, constraints):
        return None
    def _getLinDmpPD(self, constraints):
        return None
    def _getAngDmpPD(self, constraints):
        return None

    def isExplicitObject(self, obj):
        if obj._parameters["physics_type"] in ["rigid_body", "soft_body"]:
            if list(set(["is_kinematic", "has_collision", "physics_type", "name"]) - set(obj._parameters.keys())):
                return False
        if obj._parameters["physics_type"] in ["rigid_body", "soft_body"]:
            if list(set(self._getParamset("translation") + self._getParamset("rotation") + self._getParamset("velocity") + self._getParamset("angular_velocity")) - set(obj._parameters.keys())):
                return False
        if obj._parameters["physics_type"] in ["rigid_body", "soft_body"]:
            if list(set(["friction", "restitution", "mesh", "mass", "linear_damping", "angular_damping"]).difference(set(obj._parameters.keys()))):
                return False
        if "soft_body" == obj._parameters["physics_type"]:
            if set(["stretch_resistance", "compress_resistance", "bend_resistance", "damping", "plasticity"]).difference(set(obj._parameters.keys())):
                return False
        if "impulse_particles" == obj._parameters["physics_type"]:
            if ("particles" not in obj._parameters.keys()) or (not obj._parameters["particles"]):
                return False
            for p in obj._parameters["particles"]:
                if set(self._getParamset("translation") + self._getParamset("velocity")).difference(p.keys()):
                    return False
                if set(["friction", "restitution", "mesh", "mass", "linear_damping", "angular_damping"]).difference(p.keys()):
                    return False
        return True
    def isExplicatableSchema(self, schema):
        # the following schemas do not require further theory to elucidate, as far as a physics simulator is concerned:
        if schema._type in ["Object", "ParticleSystem", "Var", "SurfaceNormal", "WorldVerticalDirection", "WorldRelativeTopSurface", "WorldRelativeBottomSurface", "ObjectRelativeTopSurface", "ObjectRelativeBottomSurface", "UprightDirection", "Centroid", "Interior", "CollisionEnabled", "CollisionDisabled", "SurfaceContainment", "PointInVolume", "AxisAlignment", "AxisCounterAlignment", "AxisOrthogonality", "RelativeDepart", "RelativeStay", "RelativeStayLevel", "RelativeFall", "RelativeApproach", "Expectation", "PathAbsence", "PathExistence"]:
            return True
        return False

