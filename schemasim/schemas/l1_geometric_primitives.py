import os
import sys

import schemasim.util.object_semantic_data as osd

import schemasim.schemas.l0_schema_templates as st

from schemasim.util.geometry import poseFromTQ

import trimesh

import numpy as np

class PointPrimitive(st.RoleDefiningSchema):
    def __init__(self):
        super().__init__()
        self._type = "PointPrimitive"
        self._roles = {}
    def getPoint(self):
        return [0.0,0.0,0.0]

class AxisPrimitive(st.RoleDefiningSchema):
    def __init__(self):
        super().__init__()
        self._type = "AxisPrimitive"
        self._roles = {}
    def getAxis(self):
        return [0.0,0.0,1.0]

class SurfacePrimitive(st.RoleDefiningSchema):
    def __init__(self):
        super().__init__()
        self._type = "SurfacePrimitive"
        self._roles = {}
    def _getVolumeInternal(self):
        if (not isinstance(self._roles["obj"], st.ParameterizedSchema)):
            return None
        meshPath = self._roles["obj"].getMeshPath(modifier=".stl")
        if not meshPath:
            return None
        return trimesh.load(meshPath)
    def _getTransformInternal(self):
        t = [0.0,0.0,0.0]
        q = [0.0,0.0,0.0,1.0]
        j = 0
        for p in ["tx", "ty", "tz"]:
            if p in self._roles["obj"]._parameters:
                t[j] = self._roles["obj"]._parameters[p]
            j = j + 1
        j = 0
        for p in ["rx", "ry", "rz", "rw"]:
            if p in self._roles["obj"]._parameters:
                q[j] = self._roles["obj"]._parameters[p]
            j = j + 1
        return t, q
    def getNormal(self):
        return [0.0,0.0,1.0]
    def getSurface(self):
        return [[0.0,0.0,0.0]]
    def getSurfaceBounds(self):
        mins = [1000000.0, 1000000.0, 1000000.0]
        maxs = [-1000000.0, -1000000.0, -1000000.0]
        points = self.getSurface()
        if not points:
            return None, None, None
        for p in points:
            for k in [0,1,2]:
                if p[k] < mins[k]:
                    mins[k] = p[k]
                if maxs[k] < p[k]:
                    maxs[k] = p[k]
        return [[mins[0], maxs[0]], [mins[1], maxs[1]], [mins[2], maxs[2]]], [0.0,0.0,0.0], [0.0,0.0,0.0,1.0]

class WorldVerticalDirection(AxisPrimitive):
    def __init__(self):
        super().__init__()
        self._type = "WorldVerticalDirection"
        self._roles = {}
    def getAxis(self):
        return [0.0,0.0,1.0]

class WorldRelativeTopSurface(SurfacePrimitive):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "WorldRelativeTopSurface"
        self._roles = {"obj": obj}
    def getNormal(self):
        return [0.0,0.0,1.0]
    def getSurface(self):
        volume = self._getVolumeInternal()
        t, q = self._getTransformInternal()
        volume.apply_transform(poseFromTQ(t,q))
        volumeRayIntersector = trimesh.ray.ray_triangle.RayMeshIntersector(volume)
        bbox = list(volume.bounds)
        bbox = [[bbox[0][0], bbox[1][0]], [bbox[0][1], bbox[1][1]], [bbox[0][2], bbox[1][2]]]
        meshCenter = list(volume.centroid)
        rayDir = [0,0,-1]
        xs = np.arange(bbox[0][0], bbox[0][1], 0.4).tolist()
        ys = np.arange(bbox[1][0], bbox[1][1], 0.4).tolist()
        getRC = lambda x,y: [x, y, bbox[2][1] + 0.03]
        r2c = {}
        ray_dirs = []
        ray_centers = []
        k = 0
        for y in ys:
            for x in xs:
                ray_dirs.append(rayDir)
                ray_centers.append(getRC(x, y))
                r2c[k] = [x, y, bbox[2][1]]
                k = k + 1
        triangs, rays = volumeRayIntersector.intersects_id(ray_centers, ray_dirs, multiple_hits=False)
        rays = list(set(list(rays)))
        retq = []
        for r in rays:
            retq.append(r2c[r])
        return retq

class ObjectRelativeBottomSurface(SurfacePrimitive):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "ObjectRelativeBottomSurface"
        self._roles = {"obj": obj}
    def getNormal(self):
        if isinstance(self._roles["obj"], st.ParameterizedSchema):
            semPath = self._roles["obj"].getMeshPath(modifier=".sem")
            if semPath:
                bsn = osd.queryObjectSemanticData(semPath, "bottom_surface_normal")
                if bsn:
                    return bsn
        return [0.0,0.0,-1.0]
    def getSurface(self):
        volume = self._getVolumeInternal()
        volumeRayIntersector = trimesh.ray.ray_triangle.RayMeshIntersector(volume)
        bbox = list(volume.bounds)
        bbox = [[bbox[0][0], bbox[1][0]], [bbox[0][1], bbox[1][1]], [bbox[0][2], bbox[1][2]]]
        meshCenter = list(volume.centroid)
        rayDir = [0,0,1]
        xs = np.arange(bbox[0][0], bbox[0][1], 0.3).tolist()
        ys = np.arange(bbox[1][0], bbox[1][1], 0.3).tolist()
        getRC = lambda x,y: [x, y, bbox[2][0] - 0.1]
        r2c = {}
        ray_dirs = []
        ray_centers = []
        k = 0
        for y in ys:
            for x in xs:
                ray_dirs.append(rayDir)
                ray_centers.append(getRC(x, y))
                r2c[k] = [x, y, bbox[2][0]-0.005]
                k = k + 1
        triangs, rays = volumeRayIntersector.intersects_id(ray_centers, ray_dirs, multiple_hits=False)
        rays = list(set(list(rays)))
        retq = []
        for r in rays:
            retq.append(r2c[r])
        return retq

class UprightDirection(st.RoleDefiningSchema):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "UprightDirection"
        self._roles = {"obj": obj}
    def getAxis(self):
        if isinstance(self._roles["obj"], st.ParameterizedSchema):
            semPath = self._roles["obj"].getMeshPath(modifier=".sem")
            if semPath:
                ud = osd.queryObjectSemanticData(semPath, "upright_direction")
                if ud:
                    return ud
        return [0.0,0.0,1.0]

class SurfaceNormal(st.RoleDefiningSchema):
    def __init__(self, surface=None):
        super().__init__()
        self._type = "SurfaceNormal"
        self._roles = {"surface": surface}
    def getAxis(self):
        return self._roles["surface"].getNormal()

class Centroid(PointPrimitive):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "Centroid"
        self._roles = {"obj": obj}
    def getPoint(self):
        point = [0.0,0.0,0.0]
        obj = self._roles["obj"]
        if isinstance(obj, st.ParameterizedSchema):
            meshPath = obj.getMeshPath(modifier=".stl")
            if not meshPath:
                return None
            mesh = trimesh.load(meshPath)
            point = list(mesh.centroid)
            havePose = False
            t = [0,0,0]
            k = 0
            for p in ["tx", "ty", "tz"]:
                if p in obj._parameters:
                    havePose = True
                    t[k] = obj._parameters[p]
                k = k + 1
            if havePose:
                point[0] = point[0] + t[0]
                point[1] = point[1] + t[1]
                point[2] = point[2] + t[2]
        else:
            return point
        
        return point

class Interior(st.RoleDefiningSchema):
    def __init__(self, obj=None):
        super().__init__()
        self._type = "Interior"
        self._roles = {"obj": obj}
    def _getVolumeInternal(self):
        if (not isinstance(self._roles["obj"],st.ParameterizedSchema)):
            return None
        interiorPath = self._roles["obj"].getMeshPath(modifier="_interior.stl")
        if not interiorPath:
            return None
        return trimesh.load(interiorPath)
    def _getTransformInternal(self):
        t = [0.0,0.0,0.0]
        q = [0.0,0.0,0.0,1.0]
        j = 0
        for p in ["tx", "ty", "tz"]:
            if p in self._roles["obj"]._parameters:
                t[j] = self._roles["obj"]._parameters[p]
            j = j + 1
        j = 0
        for p in ["rx", "ry", "rz", "rw"]:
            if p in self._roles["obj"]._parameters:
                q[j] = self._roles["obj"]._parameters[p]
            j = j + 1
        return t, q
    def getVolumeBounds(self):
        interior = self._getVolumeInternal()
        if not interior:
            return None, None, None
        t, q = self._getTransformInternal()
        bbox = list(interior.bounds)
        bbox = [[bbox[0][0], bbox[1][0]], [bbox[0][1], bbox[1][1]], [bbox[0][2], bbox[1][2]]]
        return bbox, t, q
    def getVolume(self):
        interior = self._getVolumeInternal()
        if not interior:
            return None
        t, q = self._getTransformInternal()
        transform = poseFromTQ(t, q)
        interior.apply_transform(transform)
        return interior
    def getVolumeAtFrame(self, frameData, frame):
        if 0 == frame:
            return self.getVolume()
        volume = self._getVolumeInternal()
        objectFrame = frameData[frame][self._roles["obj"].getId()]
        t = [objectFrame["x"], objectFrame["y"], objectFrame["z"]]
        q = [objectFrame["rx"], objectFrame["ry"], objectFrame["rz"], objectFrame["rw"]]
        volume.apply_transform(poseFromTQ(t, q))
        return volume

