import os
import sys

import math

import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l2_geometric_primitive_relations as gpr

from schemasim.util.geometry import volumeInclusion

class PrimitiveMovement(st.RoleDefiningSchema):
    def __init__(self, obj=None, relatum=None):
        super().__init__()
        self._type = "PrimitiveMovement"
        self._roles = {"obj": obj, "relatum": relatum}
        self._minimize_cost = True
    def _getNormalMovement(self, extents):
        return math.sqrt(extents[0]*extents[0] + extents[1]*extents[1] + extents[2]*extents[2])
    def _getDisplacementChange(self, cobj, crel, lastDisplacement):
        px = (cobj[0] - crel[0]) - lastDisplacement[0]
        py = (cobj[1] - crel[1]) - lastDisplacement[1]
        pz = (cobj[2] - crel[2]) - lastDisplacement[2]
        norm = math.sqrt(px*px + py*py + pz*pz)
        return norm, [px, py, pz]
    def _getDisplacement(self, cobj, crel):
        return self._getDisplacementChange(cobj, crel, [0.0, 0.0, 0.0])
    def _getCostDelta(self, objectvolume, relatumvolume, cobj, crel, lastDisplacement, lastDisplacementNorm):
        retq = 0.0
        if not volumeInclusion(objectvolume, relatumvolume):
            retq, disp = self._getDisplacementChange(cobj, crel, lastDisplacement)
        return retq
    def evaluateTimeline(self, frameData):
        ovolume = self._roles["obj"].getVolume()
        if not isinstance(ovolume, list):
            cost = [0.0]
            lastDisplacement = [None]
            lastDisplacementNorm = [0.0]
            extents = list(ovolume.extents)
        else:
            cost = [0.0]*len(ovolume)
            lastDisplacement = [None]*len(ovolume)
            lastDisplacementNorm = [0.0]*len(ovolume)
            extents = list(ovolume[0].extents)
        normalMovement = self._getNormalMovement(extents)
        if isinstance(self._roles["obj"], st.ParticleSystem):
            normalMovement = normalMovement*10
        for f in list(range(len(frameData))):
            if not frameData[f]:
                continue
            ovolume = self._roles["obj"].getVolumeAtFrame(frameData, f)
            rvolume = self._roles["relatum"].getVolumeAtFrame(frameData, f)
            if not isinstance(ovolume, list):
                co = [list(ovolume.centroid)]
                ovolume = [ovolume]
            else:
                co = []
                for v in ovolume:
                    co.append(list(v.centroid))
            cr = list(rvolume.centroid)
            for k in list(range(len(lastDisplacement))):
                if lastDisplacement[k]:
                    cost[k] = cost[k] + self._getCostDelta(ovolume[k], rvolume, co[k], cr, lastDisplacement[k], lastDisplacementNorm[k])
                lastDisplacementNorm[k], lastDisplacement[k] = self._getDisplacement(co[k], cr)
        for k in list(range(len(cost))):
            cost[k] = cost[k]/normalMovement
        judgement = True
        fails = 0.0
        for c in cost:
            if ((self._minimize_cost) and (0.1 < c)) or ((not self._minimize_cost) and (0.1 > c)):
                fails = fails + 1.0
        if 0.1 < fails/len(cost):
            judgement = False
        return judgement, cost

class RelativeDepart(PrimitiveMovement):
    def __init__(self, obj=None, relatum=None):
        super().__init__(obj=obj, relatum=relatum)
        self._type = "RelativeDepart"
        self._minimize_cost = True
    def _getCostDelta(self, objectvolume, relatumvolume, cobj, crel, lastDisplacement, lastDisplacementNorm):
        retq = 0.0
        if not volumeInclusion(objectvolume, relatumvolume):
            d, v = self._getDisplacement(cobj, crel)
            if d > lastDisplacementNorm:
                retq = math.fabs(d - lastDisplacementNorm)
        return retq

class RelativeApproach(PrimitiveMovement):
    def __init__(self, obj=None, relatum=None):
        super().__init__(obj=obj, relatum=relatum)
        self._type = "RelativeApproach"
        self._minimize_cost = True
    def _getCostDelta(self, objectvolume, relatumvolume, cobj, crel, lastDisplacement, lastDisplacementNorm):
        retq = 0.0
        if not volumeInclusion(objectvolume, relatumvolume):
            d, v = self._getDisplacement(cobj, crel)
            if d < lastDisplacementNorm:
                retq = math.fabs(d - lastDisplacementNorm)
        return retq

class RelativeStillness(PrimitiveMovement):
    def __init__(self, obj=None, relatum=None):
        super().__init__(obj=obj, relatum=relatum)
        self._type = "RelativeStillness"
        self._minimize_cost = True

class RelativeMovement(PrimitiveMovement):
    def __init__(self, obj=None, relatum=None):
        super().__init__(obj=obj, relatum=relatum)
        self._type = "RelativeMovement"
        self._minimize_cost = False

class PrimitiveVerticalMovement(PrimitiveMovement):
    def __init__(self, obj=None, relatum=None):
        super().__init__(obj=obj, relatum=relatum)
        self._type = "PrimitiveVerticalMovement"
    def _getNormalMovement(self, extents):
        return extents[2]

class RelativeStayLevel(PrimitiveVerticalMovement):
    def __init__(self, obj=None, relatum=None):
        super().__init__(obj=obj, relatum=relatum)
        self._type = "RelativeStayLevel"
        self._minimize_cost = True
    def _getCostDelta(self, objectvolume, relatumvolume, cobj, crel, lastDisplacement, lastDisplacementNorm):
        d, v = self._getDisplacementChange(cobj, crel, lastDisplacement)
        return math.fabs(v[2])

class RelativeFall(PrimitiveVerticalMovement):
    def __init__(self, obj=None, relatum=None):
        super().__init__(obj=obj, relatum=relatum)
        self._type = "RelativeFall"
        self._minimize_cost = False
    def _getCostDelta(self, objectvolume, relatumvolume, cobj, crel, lastDisplacement, lastDisplacementNorm):
        retq = 0.0
        d, v = self._getDisplacementChange(cobj, crel, lastDisplacement)
        if v[2] < 0.0:
            retq = math.fabs(v[2])
        return retq

