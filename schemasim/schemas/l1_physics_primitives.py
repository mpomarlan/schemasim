import os
import sys

import schemasim.schemas.l0_schema_templates as st

class CollisionEnabled(st.RoleDefiningSchema):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "CollisionEnabled"
        self._roles = {"obj": obj}

class CollisionDisabled(st.RoleDefiningSchema):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "CollisionDisabled"
        self._roles = {"obj": obj}

