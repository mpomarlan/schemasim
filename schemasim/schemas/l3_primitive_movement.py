import os
import sys

import math

import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l2_geometric_primitive_relations as gpr

from schemasim.util.geometry import volumeInclusion

class RelativeDepart(st.RoleDefiningSchema):
    def __init__(self, obj=None, relatum=None):
        super().__init__()
        self._type = "RelativeDepart"
        self._roles = {"obj": obj, "relatum": relatum}
    def evaluateTimeline(self, frameData):
        ovolume = self._roles["obj"].getVolume()
        if not isinstance(ovolume, list):
            cost = [0.0]
            last = [None]
            extents = list(ovolume.extents)
        else:
            cost = [0.0]*len(ovolume)
            last = [None]*len(ovolume)
            extents = list(ovolume[0].extents)
        normalMovement = math.sqrt(extents[0]*extents[0] + extents[1]*extents[1] + extents[2]*extents[2])
        if isinstance(self._roles["obj"], st.ParticleSystem):
            normalMovement = normalMovement*10
        for f in list(range(len(frameData)+1)):
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
            for k in list(range(len(last))):
                if not last[k]:
                    last[k] = [co[k][0] - cr[0], co[k][1] - cr[1], co[k][2] - cr[2]]
                else:
                    if not volumeInclusion(ovolume[k], rvolume):
                        px = (co[k][0] - cr[0])
                        py = (co[k][1] - cr[1])
                        pz = (co[k][2] - cr[2])
                        d = math.sqrt(px*px + py*py + pz*pz)
                        l = math.sqrt(last[k][0]*last[k][0] + last[k][1]*last[k][1] + last[k][2]*last[k][2])
                        if d > l:
                            cost[k] = cost[k] + math.fabs(d - l)
                    last[k] = [co[k][0] - cr[0], co[k][1] - cr[1], co[k][2] - cr[2]]
        for k in list(range(len(cost))):
            cost[k] = cost[k]/normalMovement
        judgement = True
        fails = 0.0
        for c in cost:
            if 0.1 < c:
                fails = fails + 1.0
        if 0.1 < fails/len(cost):
            judgement = False
        return judgement, cost

class RelativeApproach(st.RoleDefiningSchema):
    def __init__(self, obj=None, relatum=None):
        super().__init__()
        self._type = "RelativeApproach"
        self._roles = {"obj": obj, "relatum": relatum}
    def evaluateTimeline(self, frameData):
        ovolume = self._roles["obj"].getVolume()
        if not isinstance(ovolume, list):
            cost = [0.0]
            last = [None]
            extents = list(ovolume.extents)
        else:
            cost = [0.0]*len(ovolume)
            last = [None]*len(ovolume)
            extents = list(ovolume[0].extents)
        normalMovement = math.sqrt(extents[0]*extents[0] + extents[1]*extents[1] + extents[2]*extents[2])
        if isinstance(self._roles["obj"], st.ParticleSystem):
            normalMovement = normalMovement*10
        for f in list(range(len(frameData)+1)):
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
            for k in list(range(len(last))):
                if not last[k]:
                    last[k] = [co[k][0] - cr[0], co[k][1] - cr[1], co[k][2] - cr[2]]
                else:
                    if not volumeInclusion(ovolume[k], rvolume):
                        px = (co[k][0] - cr[0])
                        py = (co[k][1] - cr[1])
                        pz = (co[k][2] - cr[2])
                        d = math.sqrt(px*px + py*py + pz*pz)
                        l = math.sqrt(last[k][0]*last[k][0] + last[k][1]*last[k][1] + last[k][2]*last[k][2])
                        if d < l:
                            cost[k] = cost[k] + math.fabs(d - l)
                    last[k] = [co[k][0] - cr[0], co[k][1] - cr[1], co[k][2] - cr[2]]
        for k in list(range(len(cost))):
            cost[k] = cost[k]/normalMovement
        judgement = True
        fails = 0.0
        for c in cost:
            if 0.1 < c:
                fails = fails + 1.0
        if 0.1 < fails/len(cost):
            judgement = False
        return judgement, cost

class RelativeStillness(st.RoleDefiningSchema):
    def __init__(self, obj=None, relatum=None):
        super().__init__()
        self._type = "RelativeStillness"
        self._roles = {"obj": obj, "relatum": relatum}
    def evaluateTimeline(self, frameData):
        ovolume = self._roles["obj"].getVolume()
        if not isinstance(ovolume, list):
            cost = [0.0]
            last = [None]
            extents = list(ovolume.extents)
        else:
            cost = [0.0]*len(ovolume)
            last = [None]*len(ovolume)
            extents = list(ovolume[0].extents)
        normalMovement = math.sqrt(extents[0]*extents[0] + extents[1]*extents[1] + extents[2]*extents[2])
        if isinstance(self._roles["obj"], st.ParticleSystem):
            normalMovement = normalMovement*10
        for f in list(range(len(frameData)+1)):
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
            for k in list(range(len(last))):
                if not last[k]:
                    last[k] = [co[k][0] - cr[0], co[k][1] - cr[1], co[k][2] - cr[2]]
                else:
                    if not volumeInclusion(ovolume[k], rvolume):
                        dx = (co[k][0] - cr[0]) - last[k][0]
                        dy = (co[k][1] - cr[1]) - last[k][1]
                        dz = (co[k][2] - cr[2]) - last[k][2]
                        cost[k] = cost[k] + math.sqrt(dx*dx + dy*dy + dz*dz)
                    last[k] = [co[k][0] - cr[0], co[k][1] - cr[1], co[k][2] - cr[2]]
        for k in list(range(len(cost))):
            cost[k] = cost[k]/normalMovement
        judgement = True
        fails = 0.0
        for c in cost:
            if 0.1 < c:
                fails = fails + 1.0
        if 0.1 < fails/len(cost):
            judgement = False
        return judgement, cost

class RelativeMovement(st.RoleDefiningSchema):
    def __init__(self, obj=None, relatum=None):
        super().__init__()
        self._type = "RelativeMovement"
        self._roles = {"obj": obj, "relatum": relatum}
    def evaluateTimeline(self, frameData):
        ovolume = self._roles["obj"].getVolume()
        if not isinstance(ovolume, list):
            cost = [0.0]
            last = [None]
            extents = list(ovolume.extents)
        else:
            cost = [0.0]*len(ovolume)
            last = [None]*len(ovolume)
            extents = list(ovolume[0].extents)
        normalMovement = math.sqrt(extents[0]*extents[0] + extents[1]*extents[1] + extents[2]*extents[2])
        if isinstance(self._roles["obj"], st.ParticleSystem):
            normalMovement = normalMovement*10
        for f in list(range(len(frameData)+1)):
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
            for k in list(range(len(last))):
                if not last[k]:
                    last[k] = [co[k][0] - cr[0], co[k][1] - cr[1], co[k][2] - cr[2]]
                else:
                    if not volumeInclusion(ovolume[k], rvolume):
                        dx = (co[k][0] - cr[0]) - last[k][0]
                        dy = (co[k][1] - cr[1]) - last[k][1]
                        dz = (co[k][2] - cr[2]) - last[k][2]
                        cost[k] = cost[k] + math.sqrt(dx*dx + dy*dy + dz*dz)
                    last[k] = [co[k][0] - cr[0], co[k][1] - cr[1], co[k][2] - cr[2]]
        for k in list(range(len(cost))):
            cost[k] = cost[k]/normalMovement
        judgement = True
        fails = 0.0
        for c in cost:
            if c < 0.1:
                fails = fails + 1.0
        if 0.1 < fails/len(cost):
            judgement = False
        return judgement, cost

class RelativeStayLevel(st.RoleDefiningSchema):
    def __init__(self, obj=None, relatum=None):
        super().__init__()
        self._type = "RelativeStayLevel"
        self._roles = {"obj": obj, "relatum": relatum}
    def evaluateTimeline(self, frameData):
        ovolume = self._roles["obj"].getVolume()
        if not isinstance(ovolume, list):
            cost = [0.0]
            last = [None]
            extents = list(ovolume.extents)
        else:
            cost = [0.0]*len(ovolume)
            last = [None]*len(ovolume)
            extents = list(ovolume[0].extents)
        normalMovement = extents[2]
        if isinstance(self._roles["obj"], st.ParticleSystem):
            normalMovement = normalMovement*10
        for f in list(range(len(frameData)+1)):
            ovolume = self._roles["obj"].getVolumeAtFrame(frameData, f)
            rvolume = self._roles["relatum"].getVolumeAtFrame(frameData, f)
            if not isinstance(ovolume, list):
                co = [list(ovolume.centroid)]
            else:
                co = []
                for v in ovolume:
                    co.append(list(v.centroid))
            cr = list(rvolume.centroid)
            for k in list(range(len(last))):
                if not last[k]:
                    last[k] = co[k][2] - cr[2]
                else:
                    cost[k] = cost[k] + math.fabs((co[k][2] - cr[2]) - last[k])
                    last[k] = co[k][2] - cr[2]
        for k in list(range(len(cost))):
            cost[k] = cost[k]/normalMovement
        judgement = True
        fails = 0.0
        for c in cost:
            if 0.1 < c:
                fails = fails + 1.0
        if 0.1 < fails/len(cost):
            judgement = False
        return judgement, cost

class RelativeFall(st.RoleDefiningSchema):
    def __init__(self, obj=None, relatum=None):
        super().__init__()
        self._type = "RelativeFall"
        self._roles = {"obj": obj, "relatum": relatum}
    def evaluateTimeline(self, frameData):
        ovolume = self._roles["obj"].getVolume()
        if not isinstance(ovolume, list):
            cost = [0.0]
            last = [None]
            extents = list(ovolume.extents)
        else:
            cost = [0.0]*len(ovolume)
            last = [None]*len(ovolume)
            extents = list(ovolume[0].extents)
        normalMovement = extents[2]
        if isinstance(self._roles["obj"], st.ParticleSystem):
            normalMovement = normalMovement*10
        for f in list(range(len(frameData)+1)):
            ovolume = self._roles["obj"].getVolumeAtFrame(frameData, f)
            rvolume = self._roles["relatum"].getVolumeAtFrame(frameData, f)
            if not isinstance(ovolume, list):
                co = [list(ovolume.centroid)]
            else:
                co = []
                for v in ovolume:
                    co.append(list(v.centroid))
            cr = list(rvolume.centroid)
            for k in list(range(len(last))):
                if not last[k]:
                    last[k] = co[k][2] - cr[2]
                else:
                    if co[k][2] - cr[2] < last[k]:
                        cost[k] = cost[k] + math.fabs((co[k][2] - cr[2]) - last[k])
                    last[k] = co[k][2] - cr[2]
        for k in list(range(len(cost))):
            cost[k] = cost[k]/normalMovement
        judgement = True
        fails = 0.0
        for c in cost:
            if 0.1 > c:
                fails = fails + 1.0
        if 0.1 < fails/len(cost):
            judgement = False
        return judgement, cost

