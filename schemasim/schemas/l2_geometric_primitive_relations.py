import os
import sys

import math

import scipy.signal

import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp

class GeometricPrimitiveRelation(st.RoleDefiningSchema):
    def __init__(self):
        super().__init__()
        self._type = "GeometricPrimitiveRelation"
        self._meta_type.append("GeometricPrimitiveRelation")
    def evaluateFrame(self, frameData, simulator):
        return True, 1.0
    def filterPD(self, rpd, sim, strictness=0.005):
        return rpd

class AxisRelation(GeometricPrimitiveRelation):
    def __init__(self, a=None, b=None):
        super().__init__()
        self._type = "AxisRelation"
        self._meta_type.append("AxisRelation")
        self._roles = {"a": a, "b": b}
        self._targetAngle = None
    def getTargetAxis(self, sim):
        if sim.isExplicitSchema(self._roles["a"]):
            return self._roles["a"].getAxis(sim)
        if sim.isExplicitSchema(self._roles["b"]):
            return self._roles["b"].getAxis(sim)
        return sim.space().verticalAxis()
    def getMovingAxis(self, sim):
        if not sim.isExplicitSchema(self._roles["a"]):
            return self._roles["a"].getAxis(sim)
        if not sim.isExplicitSchema(self._roles["b"]):
            return self._roles["b"].getAxis(sim)
        return sim.space().verticalAxis()
    def evaluateFrame(self, frameData, sim):
        aAxis = self._roles["a"].getAxisAtFrame(frameData, sim)
        bAxis = self._roles["b"].getAxisAtFrame(frameData, sim)
        space = sim.space()
        angle = math.acos(space.vectorDotProduct(aAxis, bAxis))
        score = math.exp(-math.fabs(angle - self._targetAngle))
        return (math.exp(-math.fabs(0.1)) < score), score
    def filterPD(self, rpd, sim, strictness=0.005):
        targetAxis = self.getTargetAxis(sim)
        movingAxis = self.getMovingAxis(sim)
        space = sim.space()
        for c in rpd:
            movedAxis = space.transformVector(movingAxis, space.nullVector(), c[1])
            angle = math.acos(space.vectorDotProduct(targetAxis, movedAxis))
            c[0] = c[0]*math.exp(-math.fabs(angle - self._targetAngle)/strictness)
        return rpd

class AxisAlignment(AxisRelation):
    def __init__(self, a=None, b=None):
        super().__init__(a=a, b=b)
        self._type = "AxisAlignment"
        self._meta_type.append("AxisAlignment")
        self._targetAngle = 0.0

class AxisCounterAlignment(AxisRelation):
    def __init__(self, a=None, b=None):
        super().__init__(a=a, b=b)
        self._type = "AxisCounterAlignment"
        self._meta_type.append("AxisCounterAlignment")
        self._targetAngle = math.pi

class AxisOrthogonality(AxisRelation):
    def __init__(self, a=None, b=None):
        super().__init__(a=a, b=b)
        self._type = "AxisOrthogonality"
        self._meta_type.append("AxisOrthogonality")
        self._targetAngle = math.pi/2.0

class SurfaceContainment(GeometricPrimitiveRelation):
    def __init__(self, container_surface=None, containee_surface=None):
        super().__init__()
        self._type = "SurfaceContainment"
        self._meta_type.append("SurfaceContainment")
        self._roles = {"container_surface": container_surface, "containee_surface": containee_surface}
    def getTargetSurfaceBounds(self, sim):
        if sim.isExplicitSchema(self._roles["container_surface"]):
            return self._roles["container_surface"].getSurfaceBounds(sim)
        if sim.isExplicitSchema(self._roles["containee_surface"]):
            return self._roles["containee_surface"].getSurfaceBounds(sim)
    def getTargetSurface(self, sim):
        if sim.isExplicitSchema(self._roles["container_surface"]):
            return self._roles["container_surface"].getSurface(sim)
        if sim.isExplicitSchema(self._roles["containee_surface"]):
            return self._roles["containee_surface"].getSurface(sim)
    def getMovingSurface(self, sim):
        if not sim.isExplicitSchema(self._roles["containee_surface"]):
            return self._roles["containee_surface"].getSurface(sim)
        if not sim.isExplicitSchema(self._roles["container_surface"]):
            return self._roles["container_surface"].getSurface(sim)
    def filterPD(self, rpd, orientation, sim, strictness=0.005):
        space = sim.space()
        movingSurfaceIni = self.getMovingSurface(sim)
        movingSurface = []
        for e in movingSurfaceIni:
            movingSurface.append(space.transformVector(e, space.nullVector(), orientation))
        targetSurface = self.getTargetSurface(sim)
        if not movingSurface:
            return None
        ## TODO: replace to this
        movingArray, paddingDims, plane, normal = space.surfaceToImage(movingSurface)
        weightMoving = movingArray.sum()
        targetArray, dims, plane, normal = space.surfaceToImage(targetSurface, paddingDims=paddingDims, normal=normal)
        res = scipy.signal.fftconvolve(targetArray, movingArray, mode="same")
        for c in rpd:
            coords = space.getPointInHyperplaneCoords(c[1], plane, pixelate=True,dims=res.shape)
            if coords:
                cost = max(0, (weightMoving-res[tuple(coords)])/weightMoving)
                try:
                    # TODO: figure out why the shape looks like it does. Shouldn't this adjustment be dof-dependent?
                    # Why then is an exp necessary to produce a distribution that biases towards the center?
                    c[0] = c[0]/math.exp(cost*20)#/strictness)
                    #c[0] = c[0]*math.pow(res[tuple(coords)]/weightMoving, 3*sim.space().dof()-3)
                except OverflowError:
                    c[0] = 0
            else:
                c[0] = 0
        ## This is the old code, just for quick backup
        ## for c in rpd:
        ##     movedSurface = []
        ##     for e in movingSurface:
        ##         movedSurface.append(space.translateVector(e, c[1]))
        ##     cost = space.outerAreaFromSurface(movedSurface, targetSurface)
        ##     c[0] = c[0]/math.exp(cost/strictness)
        return rpd
    def evaluateFrame(self, frameData, simulator):
        erSurface = self._roles["container_surface"].getSurfaceAtFrame(frameData, simulator)
        eeSurface = self._roles["containee_surface"].getSurfaceAtFrame(frameData, simulator)
        cost = simulator.space().outerAreaFromSurface(eeSurface, erSurface)
        return (0.1>cost), cost

class PointInVolume(GeometricPrimitiveRelation):
    def __init__(self, container_volume=None, containee_point=None):
        super().__init__()
        self._type = "PointInVolume"
        self._meta_type.append("PointInVolume")
        self._roles = {"container_volume": container_volume, "containee_point": containee_point}
    def getMovingPoint(self, sim):
        if not sim.isExplicitSchema(self._roles["containee_point"]):
            return self._roles["containee_point"].getPoint(sim)
        return None
    def getTargetPoint(self, sim):
        if sim.isExplicitSchema(self._roles["containee_point"]):
            return self._roles["containee_point"].getPoint(sim)
        return None
    def getMovingVolume(self, sim):
        if not sim.isExplicitSchema(self._roles["container_volume"]):
            return self._roles["container_volume"].getVolume(sim)
        return None
    def getMovingVolumeBounds(self, sim):
        if not sim.isExplicitSchema(self._roles["container_volume"]):
            return self._roles["container_volume"].getVolumeBounds(sim)
        return None, None, None
    def getTargetVolume(self, sim):
        if sim.isExplicitSchema(self._roles["container_volume"]):
            return self._roles["container_volume"].getVolume(sim)
        return None
    def getTargetVolumeBounds(self, sim):
        if sim.isExplicitSchema(self._roles["container_volume"]):
            return self._roles["container_volume"].getVolumeBounds(sim)
        return None, None, None
    def filterPD(self, rpd, orientation, sim, strictness=0.005):
        space = sim.space()
        targetPoint = self.getTargetPoint(sim)
        targetVolume = self.getTargetVolume(sim)
        movingPoint = self.getMovingPoint(sim)
        movingVolume = self.getMovingVolume(sim)
        pt = False
        if (not (movingVolume and targetPoint)) and (not (targetVolume and movingPoint)):
            return None
        elif (movingVolume and targetPoint):
            pt = True
            volume = space.transformVolume(movingVolume, space.nullVector(), orientation)
            point = targetPoint
        elif (targetVolume and movingPoint):
            volume = targetVolume
            volumeRayIntersector = sim.space().makeRayVolumeIntersector(targetVolume)
            point = space.transformVector(movingPoint, space.nullVector(), orientation)
        last = space.origin()
        for c in rpd:
            current = space.vectorDifference(c[1], last)
            if pt:
                volume = space.translateVolume(volume, current)
                volumeRayIntersector = sim.space().makeRayVolumeIntersector(volume)
            else:
                point = space.translateVector(point, current)
            cost = space.distanceFromInterior(point, volume, volumeRayIntersector)
            c[0] = c[0]/math.exp(cost/strictness)
            centroid = list(volume.centroid)
            d = 2.0*space.vectorNorm(space.vectorDifference(point, centroid))/space.boundaryBoxDiameter(space.volumeBounds(volume))
            c[0] = 2.0*c[0]/(1.0 + math.pow(1.0 + d, 6*space.dof()))
            last = c[1]
        return rpd
    def evaluateFrame(self, frameData, simulator):
        erVolume = self._roles["container_volume"].getVolumeAtFrame([{}, frameData], 1, simulator)
        eePoint = self._roles["containee_point"].getPoint(simulator, frameData=frameData)
        cost = simulator.space().distanceFromInterior(eePoint, erVolume, simulator.space().makeRayVolumeIntersector(erVolume))/simulator.space().boundaryBoxDiameter(simulator.space().volumeBounds(erVolume))
        return (0.01>cost), cost

