import os
import sys

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
    def _getVolumeInternal(self, sim):
        return sim.space().loadVolume(self.getMeshPath(modifier=sim.space().volumePathModifier()))
    def _getTransformInternal(self, sim):
        return sim.translationVector(self), sim.rotationRepresentation(self)
    def getVolumeBounds(self, sim):
        t, r = self._getTransformInternal(sim)
        return sim.space().volumeBounds(self._getVolumeInternal(sim)), t, r
    def getVolume(self, sim):
        t, r = self._getTransformInternal(sim)
        return sim.space().transformVolume(self._getVolumeInternal(sim), t, r)
    def getFloorLevel(self, sim):
        return sim.space().floorLevel(self.getVolume(sim))
    def getVolumeAtFrame(self, frameData, frame, sim):
        if 0 == frame:
            return self.getVolume(sim)
        volume = self._getVolumeInternal(sim)
        objectFrame = frameData[frame][self.getId()]
        t = sim.translationVector(objectFrame)
        r = sim.rotationRepresentation(objectFrame)
        return sim.space().transformVolume(volume, t, r)
    def getTrajectories(self, frameData):
        name = self.getId()
        trajectories = {name: {}}
        k = 0
        for f in frameData:
            if name in f:
                trajectories[name][k] = f[name]
            k = k + 1
        return trajectories

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
    def _getParticleTransformsInternal(self, sim):
        if not "particles" in self._parameters:
            return None
        ts = []
        rs = []
        for p in self._parameters["particles"]:
            ts.append(sim.translationVector(p))
            rs.append(sim.rotationRepresentation(p))
        return ts, rs
    def getVolume(self, sim):
        if not "particles" in self._parameters:
            return None
        basevolume = self._getVolumeInternal(sim)
        ts, rs = self._getParticleTransformsInternal(sim)
        volumes = []
        for t, r in zip(ts, rs):
            volume = sim.space().transformVolume(basevolume.copy(), t, r)
            volumes.append(volume)
        return volumes
    def getVolumeAtFrame(self, frameData, frame, sim):
        if 0 == frame:
            return self.getVolume(sim)
        basevolume = self._getVolumeInternal(sim)
        objectFrame = self._getFrameData(frameData[frame])
        volumes = []
        for p in objectFrame:
            volumes.append(sim.space().transformVolume(basevolume.copy(), sim.translationVector(p), sim.rotationRepresentation(p)))
        return volumes
    def getTrajectories(self, frameData):
        trajectories = {}
        if not "particles" in self._parameters:
            return {}
        name = self.getId()
        k = 0
        for f in frameData:
            j = 0
            for p in self._parameters["particles"]:
                pname = name + ":" + str(j)
                if pname in f:
                    if pname not in trajectories:
                        trajectories[pname] = {}
                    trajectories[pname][k] = f[pname]
                j = j + 1
            k = k + 1
        return trajectories

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


