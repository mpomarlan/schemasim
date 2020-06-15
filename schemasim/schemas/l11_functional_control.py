import os
import sys

import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l1_physics_primitives as pp
import schemasim.schemas.l2_geometric_primitive_relations as gpr
import schemasim.schemas.l3_primitive_movement as pm
import schemasim.schemas.l3_location as location
import schemasim.schemas.l4_path as path
import schemasim.schemas.l10_expectation as expectation

class FunctionalControl(st.RoleDefiningSchema):
    def __init__(self):
        super().__init__()
        self._meta_type.append("FunctionalControl")

class Support(FunctionalControl):
    def __init__(self, supporter=None, supportee=None):
        super().__init__()
        self._type = "Support"
        self._meta_type.append("Support")
        self._roles = {"supporter": supporter, "supportee": supportee}
    def getSchemaTheory(self, schemaNet):
        retq = super().getSchemaTheory(schemaNet)
        #TODO: insert special cases here
        # the default theory: 
        #    to establish: supportee is "on" the supporter
        #                  supportee doesn't support supporter.
        #    to verify: supporter has collision -> supportee has no vertical movement;
        #               supporter has no collision -> supportee has vertical movement.
        supportee = self._roles["supportee"]
        supporter = self._roles["supporter"]
        on = location.Location(locatum=supportee,relatum=supporter,spatial_modality="on")
        ef = expectation.Expectation(condition=pp.CollisionDisabled(obj=supporter), event=pm.RelativeFall(obj=supportee, relatum=supporter))
        es = expectation.Expectation(condition=pp.CollisionEnabled(obj=supporter), event=pm.RelativeStayLevel(obj=supportee, relatum=supporter))
        retq._rules.append({"antecedent": [self], "consequent+": on, "consequent-": None})
        retq._rules.append({"antecedent": [self], "consequent+": None, "consequent-": Support(supporter=supportee, supportee=supporter)})
        retq._rules.append({"antecedent": [self], "consequent+": ef, "consequent-": None})
        retq._rules.append({"antecedent": [self], "consequent+": es, "consequent-": None})
        return retq

class Containment(FunctionalControl):
    def __init__(self, container=None, containee=None):
        super().__init__()
        self._type = "Containment"
        self._meta_type.append("Containment")
        self._roles = {"container": container, "containee": containee}
    def getSchemaTheory(self, schemaNet):
        retq = super().getSchemaTheory(schemaNet)
        #TODO: insert special cases here
        # the default theory:
        #    to establish: location of containee is inside container;
        #                  contained doesn't contain the container.
        #    to verify: container has collision: containee stays put;
        #               container has no collision: containee departs.
        containee = self._roles["containee"]
        container = self._roles["container"]
        it = location.Location(locatum=containee,relatum=container,spatial_modality="in")
        ed = expectation.Expectation(condition=pp.CollisionDisabled(obj=container), event=pm.RelativeDepart(obj=containee, relatum=gp.Interior(obj=container)))
        es = expectation.Expectation(condition=pp.CollisionEnabled(obj=container), event=pm.RelativeStillness(obj=containee, relatum=gp.Interior(obj=container)))
        retq._rules.append({"antecedent": [self], "consequent+": it, "consequent-": None})
        retq._rules.append({"antecedent": [self], "consequent+": None, "consequent-": Containment(container=containee, containee=container)})
        retq._rules.append({"antecedent": [self], "consequent+": ed, "consequent-": None})
        retq._rules.append({"antecedent": [self], "consequent+": es, "consequent-": None})
        return retq

class Coverage(FunctionalControl):
    def __init__(self, coverer=None, coveree=None):
        super().__init__()
        self._type = "Coverage"
        self._meta_type.append("Coverage")
        self._roles = {"coverer": coverer, "coveree": coveree}
    def getSchemaTheory(self, schemaNet):
        retq = super().getSchemaTheory(schemaNet)
        #TODO: insert special cases here
        # the default theory:
        #    to establish: coverer placed on coveree;
        #                  coveree doesn't cover the coverer.
        #    to verify: coverer has collision: no path to interior of coveree;
        #               coverer has no collision: path to interior of coveree.
        coveree = self._roles["coveree"]
        coverer = self._roles["coverer"]
        on = location.Location(locatum=coverer,relatum=coveree,spatial_modality="on")
        ep = expectation.Expectation(condition=pp.CollisionDisabled(obj=coverer), event=path.PathExistence(destination=gp.Interior(obj=coveree)))
        es = expectation.Expectation(condition=pp.CollisionEnabled(obj=coverer), event=path.PathAbsence(destination=gp.Interior(obj=coveree)))
        ep = expectation.Expectation(condition=pp.CollisionDisabled(obj=coveree), event=pm.RelativeMovement(obj=coverer, relatum=coveree))
        es = expectation.Expectation(condition=pp.CollisionEnabled(obj=coveree), event=pm.RelativeStillness(obj=coverer, relatum=coveree))
        retq._rules.append({"antecedent": [self], "consequent+": on, "consequent-": None})
        retq._rules.append({"antecedent": [self], "consequent+": None, "consequent-": Coverage(coverer=coveree, coveree=coverer)})
        retq._rules.append({"antecedent": [self], "consequent+": ep, "consequent-": None})
        retq._rules.append({"antecedent": [self], "consequent+": es, "consequent-": None})
        return retq
 
