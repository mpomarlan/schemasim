import os
import sys

import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l2_geometric_primitive_relations as gpr

class Location(st.RoleDefiningSchema):
    def __init__(self, locatum=None, relatum=None, spatial_modality=None):
        super().__init__()
        self._type = "Location"
        self._meta_type.append("Location")
        self._roles = {"locatum": locatum, "relatum": relatum, "spatial_modality": spatial_modality}
    def getSchemaTheory(self, schemaNet):
        retq = super().getSchemaTheory(schemaNet)
        #TODO: insert special cases here
        # the default theories: 
        relatum = self._roles["relatum"]
        locatum = self._roles["locatum"]
        if "on" == self._roles["spatial_modality"]:
        #    to establish: locatum is upright;
        #                  locatum bottom surface is inside what is now the top surface of the relatum;
        #                  relatum is not on the locatum
            sc = gpr.SurfaceContainment(container_surface=gp.WorldRelativeTopSurface(obj=relatum), containee_surface=gp.ObjectRelativeBottomSurface(obj=locatum))
            aa = gpr.AxisAlignment(a=gp.WorldVerticalDirection(), b=gp.UprightDirection(obj=locatum))
            retq._rules.append({"antecedent": [self], "consequent+": sc, "consequent-": None})
            retq._rules.append({"antecedent": [self], "consequent+": aa, "consequent-": None})
            retq._rules.append({"antecedent": [self], "consequent+": None, "consequent-": Location(locatum=relatum, relatum=locatum, spatial_modality="on")})
        elif "in" == self._roles["spatial_modality"]:
        #    to establish: locatum is placed in the Interior region of relatum;
        #                  relatum is not in the locatum
            vc = gpr.PointInVolume(container_volume=gp.Interior(obj=relatum), containee_point=gp.Centroid(obj=locatum))
            retq._rules.append({"antecedent": [self], "consequent+": vc, "consequent-": None})
            retq._rules.append({"antecedent": [self], "consequent+": None, "consequent-": Location(locatum=relatum, relatum=locatum, spatial_modality="in")})
        return retq


