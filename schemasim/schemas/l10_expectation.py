import os
import sys

import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l2_geometric_primitive_relations as gpr
import schemasim.schemas.l3_primitive_movement as pm
import schemasim.schemas.l3_location as location
import schemasim.schemas.l4_path as path

class Expectation(st.RoleDefiningSchema):
    def __init__(self, condition=None, event=None):
        super().__init__()
        self._type = "Expectation"
        self._roles = {"condition": condition, "event": event}
    def getConditionObjectName(self):
        if not self._roles["condition"] or "obj" not in self._roles["condition"]._roles:
            return ""
        obj = self._roles["condition"]._roles["obj"]
        # TODO: what if the condition has a RoleDefiningSchema as a role filler?
        return obj.getId()
    def isCounterfactual(self):
        if not self._roles["condition"]:
            return False
        condition = self._roles["condition"]
        if condition._type in "CollisionDisabled":
            return True
        # TODO: any other counterfactual conditions?
        return False
