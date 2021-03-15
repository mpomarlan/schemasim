import os
import sys

import subprocess

from datetime import datetime

import ast

import schemasim.schemas.l0_schema_templates as st
from schemasim.schemas.l1_physics_primitives import Default, CollisionDisabled
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
    def _getParameterizedSchemas(self, s):
        if not isinstance(s, st.Schema):
            return []
        if "ParameterizedSchema" in s._meta_type:
            return [s]
        elif "RoleDefiningSchema" in s._meta_type:
            retq = []
            for r, v in s._roles.items():
                retq = retq + self._getParameterizedSchemas(v)
            return retq
        return []
    def getParameterizedSchemas(self):
        retq = []
        for s in self._schemas:
            if not isinstance(s, st.Schema):
                continue
            if "ParameterizedSchema" in s._meta_type:
                retq.append(s)
            elif "RoleDefiningSchema" in s._meta_type:
                for r, v in s._roles.items():
                    retq = retq + self._getParameterizedSchemas(v)
        return retq
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
                mergedTheory._nonfacts = mergedTheory._nonfacts + t._nonfacts
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

def getExpectations(schemas, simulator):
    allFacts = SchemaNet(schemas)
    theories = []
    aux = []
    for s in schemas:
        if ("FunctionalControl" in s._meta_type) or ("Location" in s._meta_type):
            theories.append(s.getSchemaTheory(allFacts))
        elif "Expectation" in s._meta_type:
            aux.append(s)
    mergedTheory = st.SchemaTheory()
    mergedTheory._facts = schemas
    for t in theories:
        mergedTheory._rules = mergedTheory._rules + t._rules
        mergedTheory._nonfacts = mergedTheory._nonfacts + t._nonfacts
    mergedTheory.reason()
    conclusions = mergedTheory._facts
    if conclusions:
        for c in conclusions:
            if ("Expectation" in c._meta_type) or ("GeometricPrimitiveRelation" in c._meta_type):
                aux.append(c)
    retq = []
    for x in aux:
        if x not in retq:
            retq.append(x) 
    return retq

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
        collisionManager = simulator.space().makeCollisionManager()
        while k < len(explicitSchemaNet.schemas()):
            obj = explicitSchemaNet.schemas()[k]
            if "ParameterizedSchema" in obj._meta_type:
                k = k + 1
                if simulator.isExplicitObject(obj):
                    continue
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

def checkSceneExpectations(schemas, simulator, simulationLogPath, condition=Default(), start_frame=None, end_frame=None, explicated=False, test_start_frame=True):
    retq = {True: [], False:[]}
    if isinstance(schemas, SchemaNet):
        schemas = schemas.schemas()
    parameterizedSchemas = {x.getId(): x for x in SchemaNet(orig=schemas).getParameterizedSchemas()}
    frameData = [ast.literal_eval(x) for x in open(simulationLogPath).read().splitlines()][start_frame:end_frame]
    if not explicated:
        schemas = schemaExplications([], [schemas], simulator).schemas()
    if test_start_frame:
        for s in schemas:
            if "GeometricPrimitiveRelation" in s._meta_type:
                res, q = s.evaluateFrame(frameData[0], simulator)
                retq[res].append((s, q))
    defaultExpectations = {}
    counterfactualExpectations = {}
    for s in schemas:
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
    # TODO: For now, the only counterfactual condition is CollisionDisabled, but this might change in the future
    cId = None
    if "CollisionDisabled" == condition._type:
        cId = condition._roles["obj"].getId()
    currentExpectations = {}
    if cId in counterfactualExpectations:
        currentExpectations = {cId: counterfactualExpectations[cId]}
        print("counterfactual expectations for %s" % cId)
        for e in currentExpectations[cId]:
            print("\t%s" % simplePrint(e))
    for name, exps in defaultExpectations.items():
        if name != cId:
            currentExpectations[name] = exps
            print("default expectations for %s" % name)
            for e in currentExpectations[name]:
                print("\t%s" % simplePrint(e))
    for name, exps in currentExpectations.items():
        print("Evaluating expectations for %s" % name)
        for e in exps:
            judgement, cost = e._roles["event"].evaluateTimeline(frameData, simulator, parameterizedSchemas=parameterizedSchemas, disabledObjects=[cId])
            if judgement:
                print("\t%s: True (%s)" % (simplePrint(e), str(cost)))
            else:
                print("\t%s: False (%s)" % (simplePrint(e), str(cost)))
            if name not in retq:
                 retq[name] = []
            retq[name].append([e, judgement, cost])
    return retq, frameData

def interpretScene(schemas, simulator, simulate_counterfactuals=True, render=False, nframes=250, sceneFolder=None):
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
    if not sceneFolder:
        sceneFolder = os.path.join(workFolder, str(datetime.now()))
    print("Scene set up, will save results at %s" % sceneFolder)
    if not os.path.isdir(sceneFolder):
        os.mkdir(sceneFolder)
    script = simulator.sceneScript(enet.schemas(), sceneFolder, blender_filename="animation.blend", log_filename="animation.log", trajectories=None, render=render, nframes=nframes)
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
    retq = {"scene_folder": sceneFolder, "scene_results": {"default": {}, "counterfactuals": {}}, "error": None}
    retq["scene_results"]["default"], frameData = checkSceneExpectations(enet.schemas(), simulator, os.path.join(sceneFolder, "animation.log"), condition=Default(), explicated=True, test_start_frame=True)
    if simulate_counterfactuals:
        print("Analyzing counterfactual versions of the scene")
        counterfactualSceneFolder = os.path.join(sceneFolder, "counterfactuals")
        if not os.path.isdir(counterfactualSceneFolder):
            os.mkdir(counterfactualSceneFolder)
        for s in enet.schemas():
            if isinstance(s, st.ParameterizedSchema):
                cId = s.getId()
                retq["scene_results"]["counterfactuals"][cId] = {}
                hCI = s._parameters["has_collision"]
                iKn = s._parameters["is_kinematic"]
                # TODO: at the moment, the only counterfactual condition is disabling an object, but this might change ...
                s._parameters["has_collision"] = 0
                s._parameters["is_kinematic"] = 1
                trajectories = s.getTrajectories(frameData)
                cFolder = os.path.join(counterfactualSceneFolder, "disable_"+s.getId())
                if not os.path.isdir(cFolder):
                    os.mkdir(cFolder)
                script = simulator.sceneScript(enet.schemas(), cFolder, blender_filename="animation.blend", log_filename="animation.log", trajectories=trajectories, render=render, nframes=nframes)
                scriptPath = os.path.join(cFolder, "scenescript.py")
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
                    retq["error"] = ("system error when calling simulator at %s: %s" % (simPath, str(e)))
                    return retq
                print("Simulation done, will now interpret results of the counterfactual scene %s." % ("disabled_" + s.getId()))
                retq["scene_results"]["counterfactuals"][cId], frameDataCounterfactual = checkSceneExpectations(enet.schemas(), simulator, os.path.join(cFolder, "animation.log"), condition=CollisionDisabled(obj=s), explicated=True, test_start_frame=True)
                s._parameters["has_collision"] = hCI
                s._parameters["is_kinematic"] = iKn
    return retq

