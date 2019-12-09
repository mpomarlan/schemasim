import os
import sys

import trimesh

from schemasim.util.geometry import poseFromTQ

class Schema:
    def __init__(self):
        self._type = ""
        self._meta_type = ["Schema"]
    def __eq__(self, other):
        if not isinstance(other, Schema):
            return False
        return self._type == other._type

class ParameterizedSchema(Schema):
    def __init__(self):
        super().__init__()
        self._parameters = {}
        self._type = "Object"
        self._meta_type.append("ParameterizedSchema")
    def __repr__(self):
        s = self._type + "("
        for k in sorted(self._parameters.keys()):
            s = s + ("%s=%s," % (str(k), str(self._parameters[k])))
        return s + ")"
    def __eq__(self, other):
        if not super().__eq__(other):
            return False
        if not isinstance(other,ParameterizedSchema):
            return False
        # Parameter fillers should not be schemas!
        return self._parameters == other._parameters
    def getId(self):
        if "name" in self._parameters:
            return self._parameters["name"]
        if "type" in self._parameters:
            return self._parameters["type"]
        return None
    def getMeshPath(self, modifier=None):
        if "mesh" not in self._parameters:
            return None
        path = self._parameters["mesh"]
        if isinstance(modifier, str) and ("" != modifier):
            path = path[:path.rfind(".")] + modifier
        path = os.path.join("../meshes", path)
        path = os.path.join(os.path.dirname(__file__), path)
        if not os.path.isfile(path):
            return None
        return path
    def _getVolumeInternal(self):
        meshPath = self.getMeshPath(modifier=".stl")
        if not meshPath:
            return None
        return trimesh.load(meshPath)
    def _getTransformInternal(self):
        t = [0.0,0.0,0.0]
        q = [0.0,0.0,0.0,1.0]
        j = 0
        for p in ["tx", "ty", "tz"]:
            if p in self._parameters:
                t[j] = self._parameters[p]
            j = j + 1
        j = 0
        for p in ["rx", "ry", "rz", "rw"]:
            if p in self._parameters:
                q[j] = self._parameters[p]
            j = j + 1
        return t, q
    def getVolumeBounds(self):
        volume = self._getVolumeInternal()
        t, q = self._getTransformInternal()
        bbox = list(volume.bounds)
        bbox = [[bbox[0][0], bbox[1][0]], [bbox[0][1], bbox[1][1]], [bbox[0][2], bbox[1][2]]]
        return bbox, t, q
    def getVolume(self):
        volume = self._getVolumeInternal()
        t, q = self._getTransformInternal()
        transform = poseFromTQ(t, q)
        volume.apply_transform(transform)
        return volume
    def getVolumeAtFrame(self, frameData, frame):
        if 0 == frame:
            return self.getVolume()
        volume = self._getVolumeInternal()
        objectFrame = frameData[frame-1][self.getId()]
        t = [objectFrame["x"], objectFrame["y"], objectFrame["z"]]
        q = [objectFrame["rx"], objectFrame["ry"], objectFrame["rz"], objectFrame["rw"]]
        volume.apply_transform(poseFromTQ(t, q))
        return volume

class ParticleSystem(ParameterizedSchema):
    def __init__(self):
        super().__init__()
        self._parameters = {}
        self._type = "ParticleSystem"
        self._meta_type.append("ParticleSystem")
    def __repr__(self):
        s = self._type + "("
        for k in sorted(self._parameters.keys()):
            s = s + ("%s=%s," % (str(k), str(self._parameters[k])))
        return s + ")"
    def _getFrameData(self, frame):
        name = self.getId()
        particles = {}
        name = name + ":"
        for k, v in frame.items():
            if 0 == k.find(name):
                particles[int(k[k.rfind(":")+1:])] = v
        retq = []
        for k in sorted(particles.keys()):
            retq.append(particles[k])
        return retq
    def _getParticleTransformsInternal(self):
        if not "particles" in self._parameters:
            return None
        ts = []
        qs = []
        for p in self._parameters["particles"]:
            t = [0.0,0.0,0.0]
            q = [0.0,0.0,0.0,1.0]
            j = 0
            for k in ["tx", "ty", "tz"]:
                if k in p:
                    t[j] = p[k]
                j = j + 1
            j = 0
            for k in ["rx", "ry", "rz", "rw"]:
                if k in p:
                    q[j] = p[k]
                j = j + 1
            ts.append(t)
            qs.append(q)
        return ts, qs
    def getVolume(self):
        if not "particles" in self._parameters:
            return None
        basevolume = self._getVolumeInternal()
        ts, qs = self._getParticleTransformsInternal()
        volumes = []
        for t, q in zip(ts, qs):
            transform = poseFromTQ(t, q)
            volume = basevolume.copy()
            volume.apply_transform(transform)
            volumes.append(volume)
        return volumes
    def getVolumeAtFrame(self, frameData, frame):
        if 0 == frame:
            return self.getVolume()
        basevolume = self._getVolumeInternal()
        objectFrame = self._getFrameData(frameData[frame-1])
        volumes = []
        for p in objectFrame:
            volume = basevolume.copy()
            t = [p["x"], p["y"], p["z"]]
            q = [p["rx"], p["ry"], p["rz"], p["rw"]]
            volume.apply_transform(poseFromTQ(t, q))
            volumes.append(volume)
        return volumes

class VariableSchema(ParameterizedSchema):
    def __init__(self, identity=""):
        super().__init__()
        self._parameters = {"id": identity}
        self._type = "Var"
    def __repr__(self):
        s = self._type + "("
        for k in sorted(self._parameters.keys()):
            s = s + ("%s=%s," % (str(k), str(self._parameters[k])))
        return s + ")"

class SchemaTheory:
    def __init__(self):
        self._rules = []
        self._facts = []
        self._nonfacts = []
        self._contradiction = False
    def _normal_form(self):
        if self._contradiction:
            return
        nR = []
        for r in self._rules:
            if r["consequent+"] and r["consequent-"]:
                nR.append({"antecedent": r["antecedent"], "consequent+": r["consequent+"], "consequent-": None})
                nR.append({"antecedent": r["antecedent"], "consequent-": r["consequent-"], "consequent+": None})
            elif r["consequent+"] or r["consequent-"]:
                nR.append(r)
        self._rules = nR
        self._contradiction = False
        for f in self._facts:
            if f in self._nonfacts:
                self._contradiction = True
        for f in self._nonfacts:
            if f in self._facts:
                self._contradiction = True
    def _reason_internal(self):
        nR = []
        simplifiedRule = False
        for r in self._rules:
            fp = r["consequent+"]
            fn = r["consequent-"]
            if [] == r["antecedent"]:
                if (fp in self._nonfacts) or (fn in self._facts):
                    self._contradiction = True
                    return False
                if fp and (not fp in self._facts):
                    self._facts.append(fp)
                if fn and (not fn in self._nonfacts):
                    self._nonfacts.append(fn)
            elif (fp and (fp in self._facts)) or (fn and (fn in self._nonfacts)):
                continue
            elif fn in self._facts:
                r["consequent-"] = r["antecedent"][0]
                r["antecedent"] = r["antecedent"][1:]
                simplifiedRule = True
                nR.append(r)
            elif fp in self._nonfacts:
                r["consequent+"] = None
                r["consequent-"] = r["antecedent"][0]
                r["antecedent"] = r["antecedent"][1:]
                simplifiedRule = True
                nR.append(r)
            else:
                skipRule = False
                for f in self._nonfacts:
                    if f in r["antecedent"]:
                        skipRule = True
                if skipRule:
                    continue
                for f in self._facts:
                    if f in r["antecedent"]:
                        r["antecedent"].remove(f)
                        simplifiedRule = True
                nR.append(r)
        self._rules = nR
        return simplifiedRule
    def reason(self):
        self._normal_form()
        if self._contradiction:
            self._facts = []
            self._nonfacts = []
            self._rules = []
            return False
        while self._reason_internal():
            continue
        return not self._contradiction
    def facts(self):
        if self._contradiction:
            return None
        return self._facts

class RoleDefiningSchema(Schema):
    def __init__(self):
        super().__init__()
        self._roles = {}
        self._meta_type.append("RoleDefiningSchema")
    def __repr__(self):
        s = self._type + "("
        for k in sorted(self._roles.keys()):
            s = s + ("%s=%s," % (str(k), str(self._roles[k])))
        return s + ")"
    def getSchemaTheory(self, schemaNet):
        retq = SchemaTheory()
        # if the schema theory is already instantiated, then it must contain exactly one fact which is the schema itself
        retq._facts.append(self)
        # if a generic schema theory should be returned, then:
        #   there must be exactly one fact in a schema theory, and that fact must have the same type and roles as the schema; all fillers however are variables
        #varSchema = RoleDefiningSchema()
        #varSchema._type = self._type
        #for k in self._roles.keys():
        #    varSchema._roles[k] = VariableSchema(identifier=("X_%s" % k))
        return retq
    def __eq__(self, other):
        if not super().__eq__(other):
            return False
        if not isinstance(other,RoleDefiningSchema):
            return False
        # Schema roleFiller relations should not be circular!
        return self._roles == other._roles


