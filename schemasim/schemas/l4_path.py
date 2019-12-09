import os
import sys

import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l2_geometric_primitive_relations as gpr
import schemasim.schemas.l3_primitive_movement as pm
import schemasim.schemas.l3_location as location

class PathAbsence(st.RoleDefiningSchema):
    def __init__(self, source=None, destination=None):
        super().__init__()
        self._type = "PathAbsence"
        self._roles = {"source": source, "destination": destination}

class PathExistence(st.RoleDefiningSchema):
    def __init__(self, source=None, destination=None):
        super().__init__()
        self._type = "PathExistence"
        self._roles = {"source": source, "destination": destination}

