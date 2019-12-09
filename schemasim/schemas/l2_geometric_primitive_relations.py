import os
import sys

import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp

class AxisRelation(st.RoleDefiningSchema):
    def __init__(self, a=None, b=None):
        super().__init__()
        self._type = "AxisRelation"
        self._roles = {"a": a, "b": b}
    def getTargetAxis(self, sim):
        if sim.isExplicitSchema(self._roles["a"]):
            return self._roles["a"].getAxis()
        if sim.isExplicitSchema(self._roles["b"]):
            return self._roles["b"].getAxis()
        return [0,0,1]
    def getMovingAxis(self, sim):
        if not sim.isExplicitSchema(self._roles["a"]):
            return self._roles["a"].getAxis()
        if not sim.isExplicitSchema(self._roles["b"]):
            return self._roles["b"].getAxis()
        return [0,0,1]

class SurfaceContainment(st.RoleDefiningSchema):
    def __init__(self, container_surface=None, containee_surface=None):
        super().__init__()
        self._type = "SurfaceContainment"
        self._roles = {"container_surface": container_surface, "containee_surface": containee_surface}
    def getTargetSurfaceBounds(self, sim):
        if sim.isExplicitSchema(self._roles["container_surface"]):
            return self._roles["container_surface"].getSurfaceBounds()
        if sim.isExplicitSchema(self._roles["containee_surface"]):
            return self._roles["containee_surface"].getSurfaceBounds()
    def getTargetSurface(self, sim):
        if sim.isExplicitSchema(self._roles["container_surface"]):
            return self._roles["container_surface"].getSurface()
        if sim.isExplicitSchema(self._roles["containee_surface"]):
            return self._roles["containee_surface"].getSurface()
    def getMovingSurface(self, sim):
        if not sim.isExplicitSchema(self._roles["container_surface"]):
            return self._roles["container_surface"].getSurface()
        if not sim.isExplicitSchema(self._roles["containee_surface"]):
            return self._roles["containee_surface"].getSurface()

class PointInVolume(st.RoleDefiningSchema):
    def __init__(self, container_volume=None, containee_point=None):
        super().__init__()
        self._type = "PointInVolume"
        self._roles = {"container_volume": container_volume, "containee_point": containee_point}
    def getMovingPoint(self, sim):
        if not sim.isExplicitSchema(self._roles["containee_point"]):
            return self._roles["containee_point"].getPoint()
        return None
    def getTargetPoint(self, sim):
        if sim.isExplicitSchema(self._roles["containee_point"]):
            return self._roles["containee_point"].getPoint()
        return None
    def getMovingVolume(self, sim):
        if not sim.isExplicitSchema(self._roles["container_volume"]):
            return self._roles["container_volume"].getVolume()
        return None
    def getMovingVolumeBounds(self, sim):
        if not sim.isExplicitSchema(self._roles["container_volume"]):
            return self._roles["container_volume"].getVolumeBounds()
        return None, None, None
    def getTargetVolume(self, sim):
        if sim.isExplicitSchema(self._roles["container_volume"]):
            return self._roles["container_volume"].getVolume()
        return None
    def getTargetVolumeBounds(self, sim):
        if sim.isExplicitSchema(self._roles["container_volume"]):
            return self._roles["container_volume"].getVolumeBounds()
        return None, None, None

class AxisAlignment(AxisRelation):
    def __init__(self, a=None, b=None):
        super().__init__()
        self._type = "AxisAlignment"
        self._roles = {"a": a, "b": b}

class AxisCounterAlignment(AxisRelation):
    def __init__(self, a=None, b=None):
        super().__init__()
        self._type = "AxisCounterAlignment"
        self._roles = {"a": a, "b": b}

class AxisOrthogonality(AxisRelation):
    def __init__(self, a=None, b=None):
        super().__init__()
        self._type = "AxisOrthogonality"
        self._roles = {"a": a, "b": b}


