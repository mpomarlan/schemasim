import os
import sys

import schemasim.schemas.l0_schema_templates as st

class PhysicalCondition(st.RoleDefiningSchema):
    def __init__(self):
        self._type = "PhysicalCondition"
        self._roles = {}
    def isDefaultCompatible(self):
        return False

class Default(PhysicalCondition):
    def __init__(self):
        self._type = "Default"
        self._roles = {}
    def isDefaultCompatible(self):
        return True

class CollisionEnabled(Default):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "CollisionEnabled"
        self._roles = {"obj": obj}

class CollisionDisabled(PhysicalCondition):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "CollisionDisabled"
        self._roles = {"obj": obj}

