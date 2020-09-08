import os
import sys

import schemasim.util.object_semantic_data as osd

import schemasim.schemas.l0_schema_templates as st

import numpy as np

class GeometricPrimitive(st.RoleDefiningSchema):
    def __init__(self):
        super().__init__()
        self._type = "GeometricPrimitive"
        self._meta_type.append("GeometricPrimitive")
        self._roles = {}
    def _getTransformInternal(self, sim):
        return self._roles["obj"]._getTransformInternal(sim)
    def _getMeshPathModifier(self, sim):
        return ""
    def _getVolumeInternal(self, sim):
        if (not isinstance(self._roles["obj"], st.ParameterizedSchema)):
            return None
        meshPath = self._roles["obj"].getMeshPath(modifier=self._getMeshPathModifier(sim))
        if not meshPath:
            return None
        return sim.space().loadVolume(meshPath, adjustments=self._roles["obj"]._adjustments)
    def _querySemanticEntry(self, entryName, default, sim):
        if isinstance(self._roles["obj"], st.ParameterizedSchema):
            semPath = self._roles["obj"].getMeshPath(modifier=sim.space().semanticPathModifier())
            if semPath:
                sem = osd.queryObjectSemanticData(semPath, entryName)
                if sem:
                    return sem
        return default
    def getVolume(self, sim):
        obj = self._roles["obj"]
        if obj and ("ParameterizedSchema" in obj._meta_type):
            return obj.getVolume(sim)
        return None
    def getVolumeAtFrame(self, frameData, frame, sim):
        obj = self._roles["obj"]
        if obj and ("ParameterizedSchema" in obj._meta_type):
            return obj.getVolumeAtFrame([{}, frameData[frame]], 1, sim)
        return None
    def getPoint(self, sim, frameData={}):
        point = None
        mesh = None
        if frameData:
            mesh = self.getVolumeAtFrame([{}, frameData], 1, sim)
        else:
            mesh = self.getVolume(sim)
        if mesh:
            if isinstance(mesh, list):
                point = sim.space().origin()
                for m in mesh:
                    point = sim.space().vectorSum(point, list(m.centroid))
                if 0 < len(mesh):
                    point = sim.space().vectorScale(1.0/len(mesh), point)
            else:
                point = list(mesh.centroid)
        return point

class PointPrimitive(GeometricPrimitive):
    def __init__(self):
        super().__init__()
        self._type = "PointPrimitive"
        self._meta_type.append("PointPrimitive")
        self._roles = {}

class AxisPrimitive(GeometricPrimitive):
    def __init__(self):
        super().__init__()
        self._type = "AxisPrimitive"
        self._meta_type.append("AxisPrimitive")
        self._roles = {}
    def getAxis(self, sim):
        return sim.space().verticalAxis()
    def getAxisAtFrame(self, frameData, sim):
        return sim.space().verticalAxis()

class WorldRelativeAxisPrimitive(AxisPrimitive):
    def __init__(self):
        super().__init__()
        self._type = "WorldRelativeAxisPrimitive"
        self._meta_type.append("WorldRelativeAxisPrimitive")
        self._roles = {}
    def getAxisAtFrame(self, frameData, sim):
        return self.getAxis(sim)

class ObjectRelativeAxisPrimitive(AxisPrimitive):
    def __init__(self):
        super().__init__()
        self._type = "ObjectRelativeAxisPrimitive"
        self._meta_type.append("ObjectRelativeAxisPrimitive")
        self._roles = {}
    def getAxisAtFrame(self, frameData, sim):
        axis = self.getAxis(sim)
        objectFrame = frameData[self._roles["obj"].getId()]
        return sim.space().transformVector(axis, sim.space().origin(), sim.rotationRepresentation(objectFrame))

class SurfacePrimitive(GeometricPrimitive):
    def __init__(self):
        super().__init__()
        self._type = "SurfacePrimitive"
        self._meta_type.append("SurfacePrimitive")
        self._roles = {}
    def _getMeshPathModifier(self, sim):
        return sim.space().volumePathModifier()
    def getNormal(self, sim):
        return sim.space().verticalAxis()
    def _getSurfaceRayDirOffs(self, sim):
        return None, None
    def getSurface(self, sim):
        t, r = self._getTransformInternal(sim)
        volume = sim.space().transformVolume(self._getVolumeInternal(sim), t, r)
        rayDir, rayOffs = self._getSurfaceRayDirOffs(sim)
        if not rayDir:
            return [sim.space().origin()]
        return sim.space().projectRaysOnVolume(volume, rayDir, rayOffs)
    def getSurfaceBounds(self, sim):
        return sim.space().pointCloudBounds(self.getSurface(sim)), sim.space().origin(), sim.space().identityRotation()
    def getSurfaceAtFrame(self, frameData, simulator):
        return self.getSurface(simulator)

class WorldRelativeSurfacePrimitive(SurfacePrimitive):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "WorldRelativeSurfacePrimitive"
        self._meta_type.append("WorldRelativeSurfacePrimitive")
    def getSurfaceAtFrame(self, frameData, simulator):
        return self.getSurface(simulator)

class WorldVerticalDirection(WorldRelativeAxisPrimitive):
    def __init__(self):
        super().__init__()
        self._type = "WorldVerticalDirection"
        self._meta_type.append("WorldVerticalDirection")
        self._roles = {}
    def getAxis(self, sim):
        return sim.space().verticalAxis()

class WorldRelativeTopSurface(WorldRelativeSurfacePrimitive):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "WorldRelativeTopSurface"
        self._meta_type.append("WorldRelativeTopSurface")
        self._roles = {"obj": obj}
    def getNormal(self, sim):
        return sim.space().verticalAxis()
    def _getSurfaceRayDirOffs(self, sim):
        return sim.space().vectorScale(-1.0, sim.space().verticalAxis()), sim.space().vectorScale(sim.space().collisionPadding(), sim.space().verticalAxis())

class WorldRelativeBottomSurface(WorldRelativeSurfacePrimitive):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "WorldRelativeBottomSurface"
        self._meta_type.append("WorldRelativeBottomSurface")
        self._roles = {"obj": obj}
    def getNormal(self, sim):
        return sim.space().vectorScale(-1.0, sim.space().verticalAxis())
    def _getSurfaceRayDirOffs(self, sim):
        return sim.space().vectorScale(1.0, sim.space().verticalAxis()), sim.space().vectorScale(-sim.space().collisionPadding(), sim.space().verticalAxis())

class ObjectRelativeSurfacePrimitive(SurfacePrimitive):
    def __init__(self):
        super().__init__()
        self._type = "ObjectRelativeSurfacePrimitive"
        self._meta_type.append("ObjectRelativeSurfacePrimitive")
        self._roles = {}
    def _semanticEntry(self, sim):
        return None
    def _defaultNormal(self, sim):
        return sim.space().verticalAxis()
    def getNormal(self, sim):
        return self._querySemanticEntry(self._semanticEntry(sim), self._defaultNormal(sim), sim)
    def getSurfaceAtFrame(self, frameData, simulator):
        volume = self._getVolumeInternal(simulator)
        rayDir, rayOffs = self._getSurfaceRayDirOffs(simulator)
        if not rayDir:
            surface = [simulator.space().origin()]
        surface = simulator.space().projectRaysOnVolume(volume, rayDir, rayOffs)
        objectFrame = frameData[self._roles["obj"].getId()]
        t = simulator.translationVector(objectFrame)
        r = simulator.rotationRepresentation(objectFrame)
        return [simulator.space().transformVector(s, t, r) for s in surface]

class ObjectRelativeBottomSurface(ObjectRelativeSurfacePrimitive):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "ObjectRelativeBottomSurface"
        self._meta_type.append("ObjectRelativeBottomSurface")
        self._roles = {"obj": obj}
    def _semanticEntry(self, sim):
        return "bottom_surface_normal"
    def _defaultNormal(self, sim):
        return sim.space().vectorScale(-1.0, sim.space().verticalAxis())
    def _getSurfaceRayDirOffs(self, sim):
        return sim.space().vectorScale(1.0, sim.space().verticalAxis()), sim.space().vectorScale(-sim.space().collisionPadding(), sim.space().verticalAxis())

class ObjectRelativeTopSurface(ObjectRelativeSurfacePrimitive):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "ObjectRelativeTopSurface"
        self._meta_type.append("ObjectRelativeTopSurface")
        self._roles = {"obj": obj}
    def _semanticEntry(self, sim):
        return "top_surface_normal"
    def _defaultNormal(self, sim):
        return sim.space().vectorScale(1.0, sim.space().verticalAxis())
    def _getSurfaceRayDirOffs(self, sim):
        return sim.space().vectorScale(-1.0, sim.space().verticalAxis()), sim.space().vectorScale(sim.space().collisionPadding(), sim.space().verticalAxis())

class UprightDirection(ObjectRelativeAxisPrimitive):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "UprightDirection"
        self._meta_type.append("UprightDirection")
        self._roles = {"obj": obj}
    def _semanticEntry(self, sim):
        return "upright_direction"
    def getAxis(self, sim):
        return self._querySemanticEntry(self._semanticEntry(sim), sim.space().verticalAxis(), sim)

class SurfaceNormal(ObjectRelativeAxisPrimitive):
    def __init__(self, surface=None):
        super().__init__()
        self._type = "SurfaceNormal"
        self._meta_type.append("SurfaceNormal")
        self._roles = {"surface": surface}
    def getAxis(self, sim):
        return self._roles["surface"].getNormal(sim)

class Centroid(PointPrimitive):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "Centroid"
        self._meta_type.append("Centroid")
        self._roles = {"obj": obj}

class Interior(GeometricPrimitive):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "Interior"
        self._meta_type.append("Interior")
        self._roles = {"obj": obj}
    def _getMeshPathModifier(self, sim):
        return sim.space().volumeInteriorPathModifier()
    def getVolumeBounds(self, sim):
        interior = self._getVolumeInternal(sim)
        if not interior:
            return None, None, None
        t, r = self._getTransformInternal(sim)
        return sim.space().volumeBounds(interior), t, r
    def getVolume(self, sim):
        interior = self._getVolumeInternal(sim)
        if not interior:
            return None
        t, r = self._getTransformInternal(sim)
        return sim.space().transformVolume(interior, t, r)
    def getVolumeAtFrame(self, frameData, frame, sim):
        if 0 == frame:
            return self.getVolume(sim)
        volume = self._getVolumeInternal(sim)
        objectFrame = frameData[frame][self._roles["obj"].getId()]
        t = sim.translationVector(objectFrame)
        r = sim.rotationRepresentation(objectFrame)
        return sim.space().transformVolume(volume, t, r)

