import os
import sys

import trimesh

import subprocess

from datetime import datetime

import ast

import schemasim.schemas.l0_schema_templates as st
#import schemasim.schemas.l1_geometric_primitives as gp
#import schemasim.schemas.l2_geometric_primitive_relations as gpr
#import schemasim.schemas.l3_primitive_movement as pm
#import schemasim.schemas.l3_location as location
#import schemasim.schemas.l4_path as path
#import schemasim.schemas.l10_expectation as expectation

import logging.config
logging.config.dictConfig({
    'version': 1,
    # Other configs ...
    'disable_existing_loggers': True
})

class DummyCollisionManager():
    def __init__(self):
        self._objs = []
    def in_collision_single(self, mesh, pose):
        return False
    def add_object(self, name, mesh, pose):
        if name not in self._objs:
            self._objs.append(name)

class SchemaNet:
    def __init__(self, orig=None):
        self._schemas = []
        self._toposorted = False
        self._name = ""
        if orig:
            if isinstance(orig, list):
                self._schemas = list(orig)
                self._toposorted = False
                self._name = "Copy"
            else:
                self._schemas = list(orig._schemas)
                self._toposorted = orig._toposorted
                self._name = "Copy:" + orig._name
    def name(self):
        return self._name
    def schemas(self):
        return self._schemas
    def addSchemas(self, schemas):
        added = False
        for newSchema in schemas:
            if not newSchema in self:
                self._schemas.append(newSchema)
                added = True
        if added:
            self._toposorted = False
    def extractExplicitSchemas(self, simulator):
        parS = []
        explicitParS = []
        roleS = []
        explicitRoleS = []
        for s in self._schemas:
            if "ParameterizedSchema" in s._meta_type:
                if simulator.isExplicitObject(s):
                    explicitParS.append(s)
                else:
                    parS.append(s)
            else:
                if simulator.isExplicitSchema(s):
                    explicitRoleS.append(s)
                else:
                    roleS.append(s)
        self._toposorted = False
        self._schemas = parS + roleS
        retq = SchemaNet()
        retq.addSchemas(explicitParS + explicitRoleS)
        return retq
    def _depset(self, unsortedParS, roleSchema):
        if "RoleDefiningSchema" not in roleSchema._meta_type:
            return []
        retq = []
        for k,v in roleSchema._roles.items():
            if hasattr(v, "_meta_type"):
                if "RoleDefiningSchema" in v._meta_type:
                    retq = retq + [x for x in self._depset(unsortedParS, v) if x not in retq]
                elif "ParameterizedSchema" in v._meta_type and (v not in retq) and (v in unsortedParS):
                    retq.append(v)
        return retq
    def _schema_lists_equal(self, a, b):
        for x in a:
            if x not in b:
                return False
        for x in b:
            if x not in a:
                return False
        return True
    def _toposort_internal(self, unsortedParS, unsortedRoleS, sortedSchemas):
        if not unsortedParS:
            return sortedSchemas + unsortedRoleS
        if not unsortedRoleS:
            return sortedSchemas + unsortedParS
        roleSO = {}
        for s in unsortedRoleS:
            deps = self._depset(unsortedParS, s)
            l = len(deps)
            if l not in roleSO:
                roleSO[l] = []
            roleSO[l].append((s, deps))
        if 0 in roleSO:
            newlySorted = [x[0] for x in roleSO[0]]
            sortedSchemas = sortedSchemas + newlySorted
            unsortedRoleS = [x for x in unsortedRoleS if x not in newlySorted]
            roleSO.pop(0)
            if not roleSO:
                return sortedSchemas + unsortedParS
        m = sorted(roleSO.keys())[0]
        mentries = roleSO[m]
        maxCount = 0
        mset = None
        for d in mentries:
            count = 0
            for e in mentries:
                if self._schema_lists_equal(d[1], e[1]):
                    count += 1
            if maxCount < count:
                maxCount = count
                mset = d[1]
        unsortedParS = [x for x in unsortedParS if x not in mset]
        sortedSchemas = sortedSchemas + mset
        return self._toposort_internal(unsortedParS, unsortedRoleS, sortedSchemas)
    def _toposort(self):
        parS = []
        roleS = []
        for s in self._schemas:
            if "ParameterizedSchema" in s._meta_type:
                parS.append(s)
            else:
                roleS.append(s)
        self._schemas = self._toposort_internal(parS, roleS, [])
        self._toposorted = True
    def popSchema(self):
        if not self._toposorted:
            self._toposort()
        popped = [self._schemas[0]]
        k = 1
        for s in self._schemas[1:]:
            if "ParameterizedSchema" in s._meta_type:
                break
            popped.append(s)
            k = k + 1
        self._schemas = self._schemas[k:]
        return popped
    def __contains__(self, schema):
        return schema in self._schemas

def schemaExplications(explicitSchemas, explicationCoreSchemas, simulator):
    retq = SchemaNet(explicitSchemas)
    for currentCoreSchemas in explicationCoreSchemas:
        while currentCoreSchemas:
            theories = []
            for s in currentCoreSchemas:
                if simulator.isExplicatableSchema(s):
                    retq.addSchemas([s])
                else:
                    theories.append(s.getSchemaTheory(retq))
            if not theories:
                break
            mergedTheory = st.SchemaTheory()
            for t in theories:
                retq.addSchemas(t._facts)
                mergedTheory._rules = mergedTheory._rules + t._rules
                mergedTheory._nonfacts = mergedTheory._facts + t._nonfacts
            mergedTheory._facts = list(retq.schemas())
            mergedTheory.reason()
            conclusions = mergedTheory._facts
            if None == conclusions:
                return SchemaNet()
            currentCoreSchemas = []
            for c in conclusions:
                if c not in retq.schemas():
                    currentCoreSchemas.append(c)
    return retq

def parameterizeObjects(schemas,simulator):
    return schemas

def explicateSchemas(schemas, simulator):
    if isinstance(schemas, list):
        schemaNet = SchemaNet()
        schemaNet.addSchemas(schemas)
        schemas = schemaNet.schemas()
    else:
        schemaNet = SchemaNet()
        schemaNet.addSchemas(schemas.schemas())
        schemas = schemaNet.schemas()
    if not simulator.compatiblePhysics(schemas):
        raise Exception('Physics in scene %s is incompatible with simulator %s' % (schemaNet.name(), self.simulator.typeName()))
    explicitSchemas = schemaNet.extractExplicitSchemas(simulator)
    if 0 == len(schemaNet.schemas()):
        # schemas are explicitly described; the list of possible explications is therefore just schemas itself
        return explicitSchemas
    else:
        # do a "topological" sort of schemas, i.e. produce a list of schemas such that they and their contexts depend on no schema still in schemas
        explicationCoreSchemas = []
        while schemaNet.schemas():
            explicationCoreSchemas.append(schemaNet.popSchema())
        explicitSchemaNet = schemaExplications(explicitSchemas, explicationCoreSchemas, simulator)
        k = 0
        try:
            collisionManager = trimesh.collision.CollisionManager()
        except ValueError:
            print("Using dummy collision manager because FCL might not be installed")
            collisionManager = DummyCollisionManager()
        while k < len(explicitSchemaNet.schemas()):
            obj = explicitSchemaNet.schemas()[k]
            if "ParameterizedSchema" in obj._meta_type:
                k = k + 1
                constraintSchemas = []
                while (k < len(explicitSchemaNet.schemas())) and ("ParameterizedSchema" not in explicitSchemaNet.schemas()[k]._meta_type):
                    if simulator.isExplicatableSchema(explicitSchemaNet.schemas()[k]):
                        constraintSchemas.append(explicitSchemaNet.schemas()[k])
                    k = k + 1
                if not simulator.parameterizeObject(obj, constraintSchemas, collisionManager):
                    return None
            else:
                k = k + 1
        return explicitSchemaNet

def simplePrint(schema):
    if isinstance(schema, st.ParameterizedSchema):
        return str(schema.getId())
    if isinstance(schema, st.RoleDefiningSchema):
        retq = schema._type + "("
        for k in sorted(schema._roles.keys()):
            retq = retq + k + "= " + simplePrint(schema._roles[k]) + ","
        return retq + ")"
    return str(schema)

def interpretScene(schemas, simulator, simulate_counterfactuals=True, render=False):
    simPath = simulator.getPath()
    if not simPath:
        print("No path known to simulator; make sure environment variable %s is set" % simulator.getPathEnvironmentVariable())
        return {"scene_results": None, "error": ("no path to simulator known because variable %s is not set" % simulator.getPathEnvironmentVariable())}
    workFolder = os.path.join(os.environ.get("HOME"), ".schemasim_scenes")
    if not os.path.isdir(workFolder):
        os.mkdir(workFolder)
    snet = SchemaNet()
    snet.addSchemas(schemas)
    print("Attempting to set up scene ...")
    enet = explicateSchemas(snet, simulator)
    if not enet:
        print("\tError: scene inconsistent or impossible.")
        return {"scene_results": None, "error": "scene inconsistent or impossible"}
    sceneFolder = os.path.join(workFolder, str(datetime.now()))
    print("Scene set up, will save results at %s" % sceneFolder)
    os.mkdir(sceneFolder)
    defaultExpectations = {}
    counterfactualExpectations = {}
    for s in enet.schemas():
        if "Expectation" == s._type:
            name = s.getConditionObjectName()
            if s.isCounterfactual():
                if name not in counterfactualExpectations:
                    counterfactualExpectations[name] = []
                counterfactualExpectations[name].append(s)
            else:
                if name not in defaultExpectations:
                    defaultExpectations[name] = []
                defaultExpectations[name].append(s)
    if defaultExpectations:
        print("Default expectations of scene:")
    for name, exps in defaultExpectations.items():
        print("\t%s" % name)
        for e in exps:
            print("\t\t%s" % simplePrint(e))
    if counterfactualExpectations:
        print("Counterfactual expectations of scene:")
    for name, exps in counterfactualExpectations.items():
        print("\t%s" % name)
        for e in exps:
            print("\t\t%s" % simplePrint(e))
    script = simulator.sceneScript(enet.schemas(), sceneFolder, blender_filename="animation.blend", log_filename="animation.log", trajectories=None, render=render)
    scriptPath = os.path.join(sceneFolder, "scenescript.py")
    with open(scriptPath, "w") as outfile:
        outfile.write(script)
    print("Generated scene setup script, will now simulate scene (this may take some time).")
    if render:
        print("\twill also render scene result, even more patience required.")
    try:
        proc = subprocess.Popen([simPath, "-b", "--python", scriptPath], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
        stdout, stderr = proc.communicate()
    except OSError as e:
        print("Encountered error when trying to call simulator:\n\t%s" % str(e))
        return {"scene_results": None, "error": ("system error when calling simulator at %s: %s" % (simPath, str(e)))}
    #if stderr:
    #    print("Simulator reported errors:\n%s" % str(stderr.decode()))
    #    return {"scene_results": None, "error": ("simulator reported errors:\n%s" % str(stderr.decode()))}
    print("Simulation done, will now interpret results of the default scene.")
    retq = {"scene_results": {"default": {}, "counterfactual": {}}, "error": None}
    frameData = [ast.literal_eval(x) for x in open(os.path.join(sceneFolder, "animation.log")).read().splitlines()]
    for name, exps in defaultExpectations.items():
        print("Evaluating expectations for %s" % name)
        for e in exps:
            judgement, cost = e._roles["event"].evaluateTimeline(frameData)
            if judgement:
                print("\t%s: True (%s)" % (simplePrint(e), str(cost)))
            else:
                print("\t%s: False (%s)" % (simplePrint(e), str(cost)))
            if name not in retq["scene_results"]["default"]:
                 retq["scene_results"]["default"][name] = []
            retq["scene_results"]["default"][name].append([e, judgement, cost])
    # TODO
    #if simulate_counterfactuals:
    #    # TODO
    #    print("Analyzing counterfactual versions of the scene")
    #    cFolder = os.path.join(sceneFolder, "counterfactuals")
    return retq

