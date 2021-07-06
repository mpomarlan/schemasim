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
        if (None != self._roles['obj']) and (self._quality in self._roles['obj']._parameters):
            retq = self._roles['obj']._parameters[self._quality]
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
    def evaluateFrame(self, frameData, sim):
        mass = self._getQuality()
        ref = self.getReferenceValue()
        sc = math.exp(-math.fabs(mass - ref)/(ref/5.0))
        return (0.2 < sc), sc
    def filterPD(self, rpd, sim, strictness=0.005):
        space = sim.space()
        ref = self.getReferenceValue()
        for c in rpd:
            c[0] = c[0]*math.exp(-math.fabs(c[1] - ref)/(ref/5.0))
        return rpd

class Heavy(MassSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "Heavy"
        self._meta_type.append("Heavy")
    def getReferenceValue(self):
        return 5*self._normal

class VeryHeavy(MassSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "VeryHeavy"
        self._meta_type.append("VeryHeavy")
    def getReferenceValue(self):
        return 25*self._normal

class Lightweight(MassSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "Lightweight"
        self._meta_type.append("Lightweight")
    def getReferenceValue(self):
        return 0.2*self._normal

class VeryLightweight(MassSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "VeryLightweight"
        self._meta_type.append("VeryLightweight")
    def getReferenceValue(self):
        return 0.04*self._normal

class RestitutionSettingSchema(PhysicsPrimitiveQuality):
    def __init__(self, obj=None):
        super().__init__(obj=obj, quality="restitution")
        self._type = "RestitutionSettingSchema"
        self._meta_type.append("RestitutionSettingSchema")
    def evaluateFrame(self, frameData, sim):
        restitution = self._getQuality()
        ref = self.getReferenceValue()
        sc = math.exp(-math.fabs(restitution - ref)/(0.1))
        return (0.2 < sc), sc
    def filterPD(self, rpd, sim, strictness=0.005):
        space = sim.space()
        ref = self.getReferenceValue()
        for c in rpd:
            c[0] = c[0]*math.exp(-math.fabs(c[1] - ref)/(0.1))
        return rpd

class Elastic(MassSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "Elastic"
        self._meta_type.append("Elastic")
    def getReferenceValue(self):
        return 0.6

class VeryElastic(MassSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "VeryElastic"
        self._meta_type.append("VeryElastic")
    def getReferenceValue(self):
        return 0.8

class Inelastic(MassSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "Inelastic"
        self._meta_type.append("Inelastic")
    def getReferenceValue(self):
        return 0.3

class VeryInelastic(MassSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "VeryInelastic"
        self._meta_type.append("VeryInelastic")
    def getReferenceValue(self):
        return 0.1

class FrictionSettingSchema(PhysicsPrimitiveQuality):
    def __init__(self, obj=None):
        super().__init__(obj=obj, quality="friction")
        self._type = "FrictionSettingSchema"
        self._meta_type.append("FrictionSettingSchema")
    def evaluateFrame(self, frameData, sim):
        friction = self._getQuality()
        ref = self.getReferenceValue()
        sc = math.exp(-math.fabs(friction - ref)/(0.1))
        return (0.2 < sc), sc
    def filterPD(self, rpd, sim, strictness=0.005):
        space = sim.space()
        ref = self.getReferenceValue()
        for c in rpd:
            c[0] = c[0]*math.exp(-math.fabs(c[1] - ref)/(0.1))
        return rpd

class Frictious(MassSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "Frictious"
        self._meta_type.append("Frictious")
    def getReferenceValue(self):
        return 0.6

class Slippery(MassSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "Slippery"
        self._meta_type.append("Slippery")
    def getReferenceValue(self):
        return 0.3

class VeryFrictious(MassSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "VeryFrictious"
        self._meta_type.append("VeryFrictious")
    def getReferenceValue(self):
        return 0.8

class VerySlippery(MassSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "VerySlippery"
        self._meta_type.append("VerySlippery")
    def getReferenceValue(self):
        return 0.1

class ParticleNumSettingSchema(PhysicsPrimitiveQuality):
    def __init__(self, obj=None):
        super().__init__(obj=obj, quality="particle_num")
        self._type = "ParticleNumSettingSchema"
        self._meta_type.append("ParticleNumSettingSchema")
        self._normal = 30
    def evaluateFrame(self, frameData, sim):
        return True, 1.0
    def filterPD(self, rpd, sim, strictness=0.005):
        space = sim.space()
        for c in rpd:
            c[0] = c[0]*math.exp(-math.fabs(c[1] - self._normal)/(self._normal/5.0))
        return rpd

class Plentiful(ParticleNumSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "Plentiful"
        self._meta_type.append("Plentiful")
        self._normal = 50

class Scarce(ParticleNumSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "Scarce"
        self._meta_type.append("Scarce")
        self._normal = 15

class VeryPlentiful(ParticleNumSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "Plentiful"
        self._meta_type.append("Plentiful")
        self._normal = 90

class VeryScarce(ParticleNumSettingSchema):
    def __init__(self, obj=None):
        super().__init__(obj=obj)
        self._type = "Scarce"
        self._meta_type.append("Scarce")
        self._normal = 5

