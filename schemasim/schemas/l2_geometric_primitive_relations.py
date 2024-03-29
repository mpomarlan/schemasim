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
    def evaluateFrame(self, frameData, sim):
        return True, 1.0
    def filterPD(self, rpd, sim, strictness=0.005):
        return rpd
    def evaluateTimeline(self, frameData, simulator, parameterizedSchemas={}, disabledObjects=[]):
        reward = 0.0
        for f in list(range(len(frameData))):
            if not frameData[f]:
                continue
            reward = reward + self.evaluateFrame(frameData[f], simulator)
        reward = reward/len(frameData)
        judgement = True
        if 0.9 > reward:
            judgement = False
        return judgement, reward

class ContactDependentRelation(GeometricPrimitiveRelation):
    def __init__(self, trajector=None, landmark=None):
        super().__init__()
        self._type = "ContactDependentRelation"
        self._meta_type.append("ContactDependentRelation")
        self._roles = {"trajector": trajector, "landmark": landmark}
    def getMovingVolume(self, sim):
        if not sim.isExplicitSchema(self._roles["trajector"]):
            return self._roles["trajector"].getVolume(sim)
        elif not sim.isExplicitSchema(self._roles["landmark"]):
            return self._roles["landmark"].getVolume(sim)
        return None
    def getTargetVolume(self, sim):
        if sim.isExplicitSchema(self._roles["trajector"]):
            return self._roles["trajector"].getVolume(sim)
        elif sim.isExplicitSchema(self._roles["landmark"]):
            return self._roles["landmark"].getVolume(sim)
        return None
    def _evaluateSignedDistance(self, d, refD):
        return True, 1.0
    def evaluateFrame(self, frameData, sim):
        a = self._roles["trajector"].getVolumeAtFrame([{}, frameData], 1, sim)
        b = self._roles["landmark"].getVolumeAtFrame([{}, frameData], 1, sim)
        d = sim.space().distanceBetweenObjects(a, b)
        refD = 0.1*sim.space().boundaryBoxDiameter(sim.space().volumeBounds(self._roles["trajector"]))
        return self._evaluateSignedDistance(d, refD)
    def filterPD(self, rpd, orientation, sim, strictness=0.005):
        refD = 0.1*sim.space().boundaryBoxDiameter(sim.space().volumeBounds(self._roles["trajector"]))
        space = sim.space()
        moving = space.transformVolume(self.getMovingVolume(sim), space.nullVector(), orientation)
        target = self.getTargetVolume(sim)
        last = space.origin()
        for c in rpd:
            current = space.vectorDifference(c[1], last)
            moving = space.translateVolume(moving, current)
            d = sim.space().distanceBetweenObjects(moving, target)
            c[0] = c[0]*self._evaluateSignedDistance(d, refD)[1]
            last = c[1]
        return rpd

class Contact(ContactDependentRelation):
    def __init__(self, trajector=None, landmark=None):
        super().__init__(a=a, b=b)
        self._type = "Contact"
        self._meta_type.append("Contact")
    def _evaluateSignedDistance(self, d, refD):
        if 0.0001 > refD:
            return False, 0.0
        if 0.0 > d:
            d = 0.0
        return (d < refD), refD/(refD+d)

class NoContact(ContactDependentRelation):
    def __init__(self, trajector=None, landmark=None):
        super().__init__(a=a, b=b)
        self._type = "NoContact"
        self._meta_type.append("NoContact")
    def _evaluateSignedDistance(self, d, refD):
        if 0.0001 > refD:
            return True, 1.0
        if 0.0 > d:
            d = 0.0
        return (d > refD), d/(refD+d)

class PointLineRelation(GeometricPrimitiveRelation):
    def __init__(self, line=None, point=None, volume=None):
        super().__init__()
        self._type = "PointLineRelation"
        self._meta_type.append("PointLineRelation")
        self._roles = {"line": line, "point": point, "volume": volume}
    def getMovingPoint(self, sim):
        if not sim.isExplicitSchema(self._roles["point"]):
            return self._roles["point"].getPoint(sim)
        return None
    def getTargetPoint(self, sim):
        if sim.isExplicitSchema(self._roles["point"]):
            return self._roles["point"].getPoint(sim)
        return None
    def getTargetLine(self, sim):
        if sim.isExplicitSchema(self._roles["line"]):
            return self._roles["line"].getLine(sim)
        return None
    def getMovingLine(self, sim):
        if not sim.isExplicitSchema(self._roles["line"]):
            return self._roles["line"].getLine(sim)
        return None
    def _evaluateDistance(self, d, ref):
        return True, 1.0
    def _getRef(self):
        ref = sim.space().translationSamplingResolution()
        if None != self._roles["volume"]:
            ref = sim.space().boundaryBoxDiameter(sim.space().volumeBounds(self._roles["volume"]))
        return ref
    def evaluateFrame(self, frameData, sim):
        point = self._roles["point"].getPoint(sim, frameData)
        line = self._roles["line"].getLineAtFrame(frameData, sim)
        if (not point) or (not line):
            return False, 0.0
        d = sim.space().distancePointLine(point, line)
        ref = self._getRef()
        return self._evaluateDistance(d, ref)
    def filterPD(self, rpd, orientation, sim, strictness=0.005):
        space = sim.space()
        ref = self._getRef()
        lineMoves = False
        point = self.getMovingPoint(sim)
        if not point:
            point = self.getTargetPoint(sim)
            line = self.getMovingLine(sim)
            a = space.transformVector(line[0:3], space.nullVector(), orientation)
            b = space.transformVector(line[3:6], space.nullVector(), orientation)
            line = a+b
            lineMoves = True
        else:
            line = self.getTargetLine(sim)
            point = space.transformVector(point, space.nullVector(), orientation)
        if (not line) or (not point):
            return None
        last = sim.space().origin()
        for c in rpd:
            tr = sim.space().vectorDifference(c[1], last)
            if lineMoves:
                pA, pB = [line[0], line[1], line[2]], [line[3], line[4], line[5]]
                pA = sim.space().vectorSum(pA, tr)
                pB = sim.space().vectorSum(pB, tr)
                line = [pA[0], pA[1], pA[2], pB[0], pB[1], pB[2]]
            else:
                point = sim.space().vectorSum(point, tr)
            c[0] = c[0]*self._evaluateDistance(d, ref)[1]
            last = c[1]
        return rpd

class PointCloseToLine(PointLineRelation):
    def __init__(self, line=None, point=None, volume=None):
        super().__init__(line=line, point=point, volume=volume)
        self._type = "PointCloseToLine"
        self._meta_type.append("PointCloseToLine")
    def _evaluateDistance(self, d, ref):
        score = ref/(ref+d)
        return (0.9 < score), score

class PointFarFromLine(PointLineRelation):
    def __init__(self, line=None, point=None, volume=None):
        super().__init__(line=line, point=point, volume=volume)
        self._type = "PointCloseToLine"
        self._meta_type.append("PointCloseToLine")
    def _evaluateDistance(self, d, ref):
        score = d/(ref+d)
        return (0.9 < score), score

class PointRelation(GeometricPrimitiveRelation):
    def __init__(self, a=None, b=None):
        super().__init__()
        self._type = "PointRelation"
        self._meta_type.append("PointRelation")
        self._roles = {"a": a, "b": b}
    def _scoreFn(self, d, D):
        return 1.0
    def getTargetPoint(self, sim):
        if sim.isExplicitSchema(self._roles["a"]):
            return self._roles["a"].getPoint(sim)
        if sim.isExplicitSchema(self._roles["b"]):
            return self._roles["b"].getPoint(sim)
        return sim.space().nullVector()
    def getMovingPoint(self, sim):
        if not sim.isExplicitSchema(self._roles["a"]):
            return self._roles["a"].getPoint(sim)
        if not sim.isExplicitSchema(self._roles["b"]):
            return self._roles["b"].getPoint(sim)
        return sim.space().nullVector()
    def evaluateFrame(self, frameData, sim):
        aPoint = self._roles["a"].getPoint(sim, frameData)
        bPoint = self._roles["b"].getPoint(sim, frameData)
        aD = sim.space().boundaryBoxDiameter(sim.space().volumeBounds(self._roles["a"].getVolume(sim)))
        bD = sim.space().boundaryBoxDiameter(sim.space().volumeBounds(self._roles["b"].getVolume(sim)))
        d = sim.space().vectorNorm(sim.space().vectorDifference(aPoint, bPoint))
        D = aD + bD
        score = self._scoreFn(d, D)
        return (self._scoreFn(0.1*D, D) < score), score
    def filterPD(self, rpd, orientation, sim, strictness=0.005):
        space = sim.space()
        rPoint = self.getTargetPoint(sim)
        mPoint = space.transformVector(self.getMovingPoint(sim), space.nullVector(), orientation)
        aD = space.boundaryBoxDiameter(space.volumeBounds(self._roles["a"].getVolume(sim)))
        bD = space.boundaryBoxDiameter(space.volumeBounds(self._roles["b"].getVolume(sim)))
        D = aD + bD
        for c in rpd:
            d = space.vectorNorm(space.vectorDifference(space.vectorSum(c[1], mPoint), rPoint))
            c[0] = c[0]*self._scoreFn(d, D)
        return rpd

class PointProximity(PointRelation):
    def __init__(self, a=None, b=None):
        super().__init__()
        self._type = "PointProximity"
        self._meta_type.append("PointProximity")
        self._roles = {"a": a, "b": b}
    def _scoreFn(self, d, D):
        xD = math.pow(D, 2)
        xd = math.pow(d, 2)
        return xD/(xD+xd)

class PointDistance(PointRelation):
    def __init__(self, a=None, b=None):
        super().__init__()
        self._type = "PointDistance"
        self._meta_type.append("PointDistance")
    def _scoreFn(self, d, D):
        xD = math.pow(D, 2)
        xd = math.pow(d, 2)
        return xd/(xD+xd)

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

class AxisDirection(GeometricPrimitiveRelation):
    def __init__(self, axis=None, point=None):
        super().__init__()
        self._type = "AxisDirection"
        self._meta_type.append("AxisDirection")
        self._roles = {"axis": axis, "point": point}
        self._targetAngle = None
    def evaluateFrame(self, frameData, sim):
        space = sim.space()
        aAxis = self._roles["axis"].getAxisAtFrame(frameData, sim)
        aPoint = self._roles["axis"].getPoint(sim, frameData)
        bPoint = self._roles["point"].getPoint(sim, frameData)
        abAxis = space.vectorNormalize(space.vectorDifference(bPoint, aPoint))
        angle = math.acos(space.vectorDotProduct(aAxis, abAxis))
        score = math.exp(-math.fabs(angle - self._targetAngle))
        return (math.exp(-math.fabs(0.1)) < score), score
    def filterPD(self, rpd, sim, strictness=0.005):
        space = sim.space()
        explicitAxis = sim.isExplicitSchema(self._roles["axis"])
        explicitPoint = sim.isExplicitSchema(self._roles["point"])
        aPoint = None
        aAxis = self._roles["axis"].getAxis(sim)
        if explicitAxis:
            aPoint = self._roles["axis"].getPoint(sim)
        bPoint = None
        if explicitPoint:
            bPoint = self._roles["point"].getPoint(sim)
        for c in rpd:
            if explicitAxis:
                bPoint = c[1]
            if explicitPoint:
                aPoint = c[1]
                aAxis = space.transformVector(self._roles["axis"].getAxis(sim), space.nullVector(), c[2])
            abAxis = space.vectorNormalize(space.vectorDifference(bPoint, aPoint))
            angle = math.acos(space.vectorDotProduct(aAxis, abAxis))
            c[0] = c[0]*math.exp(-math.fabs(angle - self._targetAngle)/strictness)
        return rpd

class AxisPointingTo(AxisDirection):
    def __init__(self, axis=None, point=None):
        super().__init__(axis=axis, point=point)
        self._type = "AxisPointingTo"
        self._meta_type.append("AxisPointingTo")
        self._targetAngle = 0

class AxisPointingAwayFrom(AxisDirection):
    def __init__(self, axis=None, point=None):
        super().__init__(axis=axis, point=point)
        self._type = "AxisPointingAwayFrom"
        self._meta_type.append("AxisPointingAwayFrom")
        self._targetAngle = math.pi

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
    def evaluateFrame(self, frameData, sim):
        erSurface = self._roles["container_surface"].getSurfaceAtFrame(frameData, sim)
        eeSurface = self._roles["containee_surface"].getSurfaceAtFrame(frameData, sim)
        cost = sim.space().outerAreaFromSurface(eeSurface, erSurface)
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
            point = space.transformVector(movingPoint, space.nullVector(), orientation)
        last = space.origin()
        for c in rpd:
            current = space.vectorDifference(c[1], last)
            if pt:
                volume = space.translateVolume(volume, current)
            else:
                point = space.translateVector(point, current)
            cost = 2.0*space.distanceFromInterior([point], volume)/sim.space().boundaryBoxDiameter(sim.space().volumeBounds(volume))
            c[0] = c[0]/math.exp(cost/strictness)
            centroid = list(volume.centroid)
            d = 2.0*space.vectorNorm(space.vectorDifference(point, centroid))/space.boundaryBoxDiameter(space.volumeBounds(volume))
            c[0] = 2.0*c[0]/(1.0 + math.pow(1.0 + d, 6*space.dof()))
            last = c[1]
        return rpd
    def evaluateFrame(self, frameData, sim):
        erVolume = self._roles["container_volume"].getVolumeAtFrame([{}, frameData], 1, sim)
        eePoint = self._roles["containee_point"].getPoint(sim, frameData=frameData)
        cost = 2.0*sim.space().distanceFromInterior([eePoint], erVolume)/sim.space().boundaryBoxDiameter(sim.space().volumeBounds(erVolume))
        return (0.2>cost), cost

class VolumePrimitiveRelation(GeometricPrimitiveRelation):
    def __init__(self, trajector=None, landmark=None):
        super().__init__()
        self._type = "VolumePrimitiveRelation"
        self._meta_type.append("VolumePrimitiveRelation")
        self._roles = {"trajector": trajector, "landmark": landmark}
    def getMovingVolume(self, sim):
        if not sim.isExplicitSchema(self._roles["trajector"]):
            return self._roles["trajector"].getVolume(sim)
        elif not sim.isExplicitSchema(self._roles["landmark"]):
            return self._roles["landmark"].getVolume(sim)
        return None
    def getTargetVolume(self, sim):
        if sim.isExplicitSchema(self._roles["landmark"]):
            return self._roles["landmark"].getVolume(sim)
        elif sim.isExplicitSchema(self._roles["trajector"]):
            return self._roles["trajector"].getVolume(sim)
        return None
    def getMovingVolumeBounds(self, sim):
        if not sim.isExplicitSchema(self._roles["trajector"]):
            return self._roles["trajector"].getVolumeBounds(sim)
        elif not sim.isExplicitSchema(self._roles["landmark"]):
            return self._roles["landmark"].getVolumeBounds(sim)
        return None, None, None
    def getTargetVolumeBounds(self, sim):
        if sim.isExplicitSchema(self._roles["landmark"]):
            return self._roles["landmark"].getVolumeBounds(sim)
        elif sim.isExplicitSchema(self._roles["trajector"]):
            return self._roles["trajector"].getVolumeBounds(sim)
        return None, None, None
    def _evaluateVolumes(self, trajector, landmark, ref):
        return True, 1.0
    def evaluateFrame(self, frameData, sim):
        trajector = self._roles["trajector"].getVolumeAtFrame([{}, frameData], 1, sim)
        landmark = self._roles["landmark"].getVolumeAtFrame([{}, frameData], 1, sim)
        return self._evaluateVolumes(trajector, landmark)
    def filterPD(self, rpd, orientation, sim, strictness=0.005):
        space = sim.space()
        targetVolume = self.getTargetVolume(sim)
        movingVolume = self.getMovingVolume(sim)
        if (not movingVolume) or (not targetVolume):
            return None
        movingTrajector = True
        if not sim.isExplicitSchema(self._roles["landmark"]):
            movingTrajector = False
        movingVolume = space.transformVolume(movingVolume, space.nullVector(), orientation)
        ref = 0.5*(sim.space().boundaryBoxDiameter(sim.space().volumeBounds(targetVolume)) + sim.space().boundaryBoxDiameter(sim.space().volumeBounds(movingVolume)))
        last = space.origin()
        for c in rpd:
            movingVolume = space.translateVolume(movingVolume, space.vectorDifference(c[1], last))
            if movingTrajector:
                c[0] = c[0]*self._evaluateVolumes(movingVolume, targetVolume, ref)
            else:
                c[0] = c[0]*self._evaluateVolumes(targetVolume, movingVolume, ref)
            last = c[1]
        return rpd

class VolumeAboveVolume(VolumePrimitiveRelation):
    def __init__(self, trajector=None, landmark=None):
        super().__init__()
        self._type = "VolumeAboveVolume"
        self._meta_type.append("VolumeAboveVolume")
        self._roles = {"trajector": trajector, "landmark": landmark}
    def _evaluateVolumes(self, trajector, landmark, ref):
        t = sim.space().volumeBounds(trajector)[2][0]
        l = sim.space().volumeBounds(landmark)[2][1]
        d = t - l
        score = 1.0/(1.0 + math.exp(0.1*ref - d))
        return (0.5 < score), score

class VolumeBelowVolume(VolumePrimitiveRelation):
    def __init__(self, trajector=None, landmark=None):
        super().__init__()
        self._type = "VolumeBelowVolume"
        self._meta_type.append("VolumeBelowVolume")
        self._roles = {"trajector": trajector, "landmark": landmark}
    def _evaluateVolumes(self, trajector, landmark, ref):
        t = sim.space().volumeBounds(trajector)[2][1]
        l = sim.space().volumeBounds(landmark)[2][0]
        d = l - t
        score = 1.0/(1.0 + math.exp(0.1*ref - d))
        return (0.5 < score), score

class VolumeInVolume(GeometricPrimitiveRelation):
    def __init__(self, container_volume=None, containee_point=None):
        super().__init__()
        self._type = "VolumeInVolume"
        self._meta_type.append("VolumeInVolume")
        self._roles = {"container": container_volume, "containee": containee_point}
    def getMovingVolume(self, sim):
        if not sim.isExplicitSchema(self._roles["containee"]):
            return self._roles["containee"].getVolume(sim)
        elif not sim.isExplicitSchema(self._roles["container"]):
            return self._roles["container"].getVolume(sim)
        return None
    def getTargetVolume(self, sim):
        if sim.isExplicitSchema(self._roles["container"]):
            return self._roles["container"].getVolume(sim)
        elif sim.isExplicitSchema(self._roles["containee"]):
            return self._roles["containee"].getVolume(sim)
        return None
    def getMovingVolumeBounds(self, sim):
        if not sim.isExplicitSchema(self._roles["containee"]):
            return self._roles["containee"].getVolumeBounds(sim)
        elif not sim.isExplicitSchema(self._roles["container"]):
            return self._roles["container"].getVolumeBounds(sim)
        return None, None, None
    def getTargetVolumeBounds(self, sim):
        if sim.isExplicitSchema(self._roles["container"]):
            return self._roles["container"].getVolumeBounds(sim)
        elif sim.isExplicitSchema(self._roles["containee"]):
            return self._roles["containee"].getVolumeBounds(sim)
        return None, None, None
    def filterPD(self, rpd, orientation, sim, strictness=0.005):
        space = sim.space()
        targetVolume = self.getTargetVolume(sim)
        movingVolume = self.getMovingVolume(sim)
        if (not movingVolume) and (not targetVolume):
            return None
        movingVolume = space.transformVolume(movingVolume, space.nullVector(), orientation)
        nF = 0.5*(sim.space().boundaryBoxDiameter(sim.space().volumeBounds(targetVolume)) + sim.space().boundaryBoxDiameter(sim.space().volumeBounds(movingVolume)))
        last = space.origin()
        for c in rpd:
            current = space.vectorDifference(c[1], last)
            movingVolume = space.translateVolume(movingVolume, current)
            cost = space.distanceFromInterior([point], volume)/nF
            c[0] = c[0]/math.exp(cost/strictness)
            tCentroid = list(targetVolume.centroid)
            mCentroid = list(movingVolume.centroid)
            d = 2.0*space.vectorNorm(space.vectorDifference(tCentroid, mCentroid))/space.boundaryBoxDiameter(space.volumeBounds(volume))
            c[0] = 2.0*c[0]/(1.0 + math.pow(1.0 + d, 6*space.dof()))
            last = c[1]
        return rpd
    def evaluateFrame(self, frameData, sim):
        erVolume = self._roles["container"].getVolumeAtFrame([{}, frameData], 1, sim)
        eeVolume = self._roles["containee"].getVolumeAtFrame([{}, frameData], 1, sim)
        nF = 0.5*(sim.space().boundaryBoxDiameter(sim.space().volumeBounds(erVolume)) + sim.space().boundaryBoxDiameter(sim.space().volumeBounds(eeVolume)))
        cost = sim.space().distanceFromInterior(eeVolume.vertices, erVolume)/nF
        return (0.2 > cost), cost

