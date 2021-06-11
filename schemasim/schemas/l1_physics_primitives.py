import os
import sys

import math

import scipy.signal

import schemasim.schemas.l0_schema_templates as st

class PhysicalCondition(st.RoleDefiningSchema):
    def __init__(self):
        super().__init__()
        self._type = "PhysicalCondition"
        self._meta_type.append("PhysicalCondition")
        self._roles = {}
    def isDefaultCompatible(self):
        return False

class Default(PhysicalCondition):
    def __init__(self):
        super().__init__()
        self._type = "Default"
        self._meta_type.append("DefaultPhysicalCondition")
        self._roles = {}
    def isDefaultCompatible(self):
        return True

class CollisionEnabled(Default):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "CollisionEnabled"
        self._meta_type.append("CollisionEnabled")
        self._roles = {"obj": obj}

class CollisionDisabled(PhysicalCondition):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "CollisionDisabled"
        self._meta_type.append("CollisionDisabled")
        self._roles = {"obj": obj}

class PhysicsPrimitiveQuality(st.RoleDefiningSchema):
    def __init__(self, obj=None, quality="", default=1.0):
        super().__init__()
        self._type = "PhysicsPrimitiveQuality"
        self._meta_type.append("PhysicsPrimitiveQuality")
        self._normal = default
        if (None != obj) and ("ParameterizedSchema" in obj._meta_type) and (quality in obj._parameters):
            self._normal = obj._parameters[quality]
        self._roles = {"obj": obj}
        self._quality = quality
    def getReferenceValue(self):
        return self._normal
    def _getQuality(self):
        retq = self._normal
        if (None != self._roles['obj']) and (quality in self._roles['obj']._parameters):
            retq = self._roles['obj']._parameters[quality]
        return retq
    def evaluateFrame(self, frameData, sim):
        return True, 1.0
    def filterPD(self, rpd, sim, strictness=0.005):
        return rpd

class MassSettingSchema(PhysicsPrimitiveQuality):
    def __init__(self, obj=None):
        super().__init__(obj=obj, quality="mass")
        self._type = "MassSettingSchema"
        self._meta_type.append("MassSettingSchema")

class Heavy(MassSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "Heavy"
        self._meta_type.append("Heavy")
    def evaluateFrame(self, frameData, sim):
        mass = self._getQuality()
        sc = math.exp(-math.fabs(c[1] - ref)/(ref/5.0))
        return (0.2 < sc), sc
    def filterPD(self, rpd, sim, strictness=0.005):
        space = sim.space()
        ref = 10*self._normal
        for c in rpd:
            movedAxis = space.transformVector(movingAxis, space.nullVector(), c[1])
            angle = math.acos(space.vectorDotProduct(targetAxis, movedAxis))
            c[0] = c[0]*math.exp(-math.fabs(c[1] - ref)/(ref/5.0))
        return rpd

