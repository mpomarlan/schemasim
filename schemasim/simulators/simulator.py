import os
import sys

import math
import numpy as np

import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l2_geometric_primitive_relations as gpr
#import schemasim.schemas.l3_primitive_movement as pm
#import schemasim.schemas.l3_location as location
#import schemasim.schemas.l4_path as path
#import schemasim.schemas.l10_expectation as expectation

import schemasim.objects.example_objects as eo

from schemasim.util.probability_density import normalizePD, samplePD

class Simulator:
    # A base class for simulators, this simulates nothing.
    def __init__(self):
        self._poseSamplingAttempts = 25
        self._space = None
        self._densityGenerationFns = {"particle_region_position": self._getParticleRegionPD, "translation": self._getTranslationPD, "rotation": self._getRotationPD, "velocity": self._getVelocityPD, "angular_velocity": self._getAngVelPD, "mesh": self._getMeshPD, "physics_type": self._getPhysicsTypePD, "particle_num": self._getParticleNumPD}
        self._assignmentFns = {"velocity": self._assignVelocity, "angular_velocity": self._assignAngularVelocity, "mesh": self._assignMesh, "physics_type": self._assignPhysicsType}
        return

    def _initializeConstraintList(self):
        return {"particle_region_position": [], "translation": [], "rotation": [], "velocity": [], "angular_velocity": [], "mesh": [], "physics_type": [], "particle_num": []}

    def _getObjParams(self, obj):
        if hasattr(obj, "_parameters"):
            params = obj._parameters
        else:
            params = obj
        return params
    def _retrieveParamset(self, obj, keyList, default):
        params = self._getObjParams(obj)
        retq = default
        k = 0
        for p in keyList:
            if p in params:
                retq[k] = params[p]
            k = k + 1
        return retq
    def translationVector(self, obj):
        return self._retrieveParamset(obj, self._getParamset("translation"), self._space.origin())
    def rotationRepresentation(self, obj):
        return self._retrieveParamset(obj, self._getParamset("rotation"), self._space.identityRotation())

    def _isIndividual(self, obj):
        return False
    def _isParticulate(self, obj):
        return False

    def _getParamset(self, parameter):
        return {"velocity": None, "angular_velocity": None, "translation": None, "rotation": None}[parameter]

    def _assignParameter(self, obj, constraints, pd, parameter):
        params = self._getObjParams(obj)
        if constraints or (parameter not in params):
            params[parameter] = samplePD(pd)
    def _assignParamset(self, obj, constraints, data, keyList):
        k = 0
        for p in keyList:
            self._assignParameter(obj, constraints, [[1.0, data[k]]], p)
            k = k + 1
    def _assignMesh(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "mesh")
    def _assignPhysicsType(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "physics_type")
    def _assignVelocity(self, obj, constraints, pd):
        if not pd:
            pd = [[1.0, self._space.nullVelocity()]]
        self._assignParamset(obj, constraints, samplePD(pd), self._getParamset("velocity"))
    def _assignAngularVelocity(self, obj, constraints, pd):
        if not pd:
            pd = [[1.0, self._space.nullAngularVelocity()]]
        self._assignParamset(obj, constraints, samplePD(pd), self._getParamset("angular_velocity"))
    def _assignTranslation(self, obj, constraints, pd):
        if not pd:
            pd = [[1.0, self._space.origin()]]
        self._assignParamset(obj, constraints, samplePD(pd), self._getParamset("translation"))
    def _assignRotation(self, obj, constraints, pd):
        if not pd:
            pd = [[1.0, self._space.identityRotation()]]
        self._assignParamset(obj, constraints, samplePD(pd), self._getParamset("rotation"))

    def _getPhysicsTypePD(self, constraints):
        return [[1.0, None]]
    def _getMeshPD(self, constraints):
        return None
    def _getParticleNumPD(self, constraints):
        return [[1.0, 30]]

    def getPathEnvironmentVariable(self):
        return "<n/a: trivial sim has no path>"
    def getPath(self):
        return os.environ.get(self.getPathEnvironmentVariable())
    def typeName(self):
        return "TrivialSimulator"
    # The empty simulator is compatible with any schema-- it won't do anything anyway.
    def compatiblePhysics(self, schemas):
        return True

    def _getParticleRegionPD(self, constraints, obj):
        pd = [[1.0, self._space.origin()]]
        if ("radius" in obj._parameters):
            r = obj._parameters["radius"]
            pd = self._space.uniformTranslationBoxRPD(self._space.cubeExtents(r), self._space.origin(), self._space.identityRotation(), resolution=self._space.particleSamplingResolution())
            for c in pd:
                d = self._space.vectorNorm(c[1])
                if r < d:
                    c[0] = 0.0
            for c in pd:
                c[1] = self._space.vectorSum(c[1], self.translationVector(obj))
        orientation = self._space.identityRotation()
        for c in constraints:
            pd = c.filterPD(pd, orientation, self, strictness=self._space.sampleValidationStrictness())
            #if "PointInVolume" == c._type:
            #    pd = c.filterByPointContainment(pd, orientation, self, strictness=self._space.sampleValidationStrictness())
            #if "SurfaceContainment" == c._type:
            #    pd = c.filterBySurfaceContainment(pd, orientation, self, strictness=self._space.sampleValidationStrictness())
        return normalizePD(pd)
    def _getTranslationPD(self, obj, constraints, orientation):
        if not constraints:
            z = -obj.getFloorLevel(self)
            return [[1.0, self._space.vectorAboveOrigin(z)]]
        pd = []
        for c in constraints:
            dims = None
            if "PointInVolume" == c._type:
                dims, translation, rotation = c.getTargetVolumeBounds(self)
                if c.getMovingVolume(self):
                    rotation = orientation
                boxPD = self._space.uniformTranslationBoxRPD(dims, translation, rotation, resolution=self._space.translationSamplingResolution())
                if c.getMovingVolume(self):
                    targetPoint = c.getTargetPoint(self)
                    for e in boxPD:
                        e[1] = self._space.vectorDifference(targetPoint, e[1])
            if "SurfaceContainment" == c._type:
                dims, translation, rotation = c.getTargetSurfaceBounds(self)
                center = self._space.centroid(c.getMovingSurface(self))
                print("CENTROID", center, orientation, c.getTargetSurface(self)[0], c.getMovingSurface(self)[0])
                center = self._space.transformVector(center, self._space.nullVector(), orientation)
                print("CENTROID", center, dims, translation, rotation)
                boxPD = self._space.uniformTranslationBoxRPD(dims, translation, rotation, resolution=2*self._space.translationSamplingResolution()) # TODO: make sc test more efficient!
                for e in boxPD:
                    e[1] = self._space.vectorDifference(e[1], center)
            if dims:
                pd = pd + boxPD
        for c in constraints:
            pd = c.filterPD(pd, orientation, self, strictness=self._space.sampleValidationStrictness())
            #if "PointInVolume" == c._type:
            #    pd = c.filterByPointContainment(pd, orientation, self, strictness=self._space.sampleValidationStrictness())
            #if "SurfaceContainment" == c._type:
            #    pd = c.filterBySurfaceContainment(pd, orientation, self, strictness=self._space.sampleValidationStrictness())
        return normalizePD(pd)
    def _getRotationPD(self, constraints):
        constraintsInterp = []
        for c in constraints:
            if isinstance(c, gpr.AxisAlignment) or isinstance(c, gpr.AxisCounterAlignment) or isinstance(c, gpr.AxisOrthogonality):
                constraintsInterp.append(c)
            elif isinstance(c, gpr.SurfaceContainment):
                constraintsInterp.append(gpr.AxisCounterAlignment(a=gp.SurfaceNormal(surface=c._roles["container_surface"]), b=gp.SurfaceNormal(surface=c._roles["containee_surface"])))
        if not constraintsInterp:
            return [[1.0, self._space.identityRotation()]]
        pd = self._space.uniformRotationRPD()
        strictness = self._space.sampleValidationStrictness()
        if 1 < len(constraintsInterp):
            strictness = strictness*len(constraintsInterp)
        for c in constraintsInterp:
            if c._type in ["AxisAlignment", "AxisCounterAlignment", "AxisOrthogonality"]:
                pd = c.filterPD(pd, self, strictness=strictness)
                #pd = c.filterByAxisAngle(pd, self, strictness=strictness)
        return normalizePD(pd)
    def _getVelocityPD(self, constraints, obj):
        pd = [[1.0, self._space.nullVelocity()]]
        if (not constraints) and (not set(["v_min", "v_max", "v_direction", "v_bias"]).difference(obj._parameters)):
            vmin = obj._parameters["v_min"]
            vmax = obj._parameters["v_max"]
            vdir = obj._parameters["v_direction"]
            vbias = obj._parameters["v_bias"]
            pd = []
            for speed in list(np.arange(vmin, vmax, self._space.speedSamplingResolution())):
                for direction in self._space.uniformDirectionSamples(samples=500, onlyPositiveQuadrant=False):
                    pd.append([1.0, self._space.vectorScale(speed, direction)])
            if 0.01 < vbias:
                for c in pd:
                    a = 0.5*self._space.vectorDotProduct(vdir, c[1]) + 0.5
                    s = math.pow(a, vbias)
                    c[0] = s*c[0]
        return normalizePD(pd)
    def _getAngVelPD(self, constraints):
        return [[1.0, self._space.nullAngularVelocity()]]

    def _sampleAndValidateObject(self,meshPath,origin,obj,collisionManager,gPDTranslation,rotation,attempts=15,addToScene=True):
        if (not gPDTranslation) or (not rotation) or (not meshPath):
            return False
        k = 0
        mesh = self._space.loadVolume(meshPath)
        while (k < attempts):
            translation = self._space.vectorSum(samplePD(gPDTranslation), origin)
            pose = self._space.poseFromTR(translation, rotation)
            if not collisionManager.in_collision_single(mesh, pose):
                name = "Object_%d" % (len(collisionManager._objs))
                if addToScene:
                    collisionManager.add_object(name, mesh, np.array(pose,dtype=np.double))
                translation = self._space.vectorDifference(translation, origin)
                self._assignTranslation(obj, [True], [[1.0, translation]])
                self._assignRotation(obj, [True], [[1.0, rotation]])
                return True
            k = k + 1
        return False
    def parameterizeObject(self, obj, constraintSchemas, collisionManager):
        # TODO: make it so that particle positions also get checked by main object translation constraints
        # These can be adjusted later once the scene is initialized, to process expectations
        obj._parameters['has_collision'] = 1
        obj._parameters['is_kinematic'] = 0
        densityMapConstraints = self._initializeConstraintList()
        for s in constraintSchemas:
            if s._type in ["AxisAlignment", "AxisCounterAlignment", "AxisOrthogonality", "SurfaceContainment"]:
                densityMapConstraints["rotation"].append(s)
            if s._type in ["PointInVolume", "SurfaceContainment"]:
                densityMapConstraints["translation"].append(s)
                densityMapConstraints["particle_region_position"].append(s)
        # Begin by selecting a mesh ...
        self._assignMesh(obj, densityMapConstraints["mesh"], self._getMeshPD(densityMapConstraints["mesh"]))
        # ... and a physics type
        self._assignPhysicsType(obj, densityMapConstraints["physics_type"], self._getPhysicsTypePD(densityMapConstraints["physics_type"]))
        densityMapConstraints.pop("mesh")
        densityMapConstraints.pop("physics_type")
        # Assign other properties depending on physics_type
        if self._isIndividual(obj):
            densityMapConstraints.pop("particle_num")
            densityMapConstraints.pop("particle_region_position")
            # Rotation and translation need special handling
            gPDOrientation = self._densityGenerationFns["rotation"](densityMapConstraints["rotation"])
            if not gPDOrientation:
                return False
            done = False
            densityMapConstraints.pop("rotation")
            for attempt in list(range(self._poseSamplingAttempts)):
                orientation = samplePD(gPDOrientation)
                gPDTranslation = self._densityGenerationFns["translation"](obj, densityMapConstraints["translation"], orientation)
                gPDVelocity = self._densityGenerationFns["velocity"](densityMapConstraints["velocity"], obj)
                if not self._sampleAndValidateObject(obj.getMeshPath(modifier=self._space.volumePathModifier()), self._space.origin(), obj, collisionManager, gPDTranslation, orientation, attempts=self._poseSamplingAttempts, addToScene=True):
                    continue
                self._assignVelocity(obj, [True], gPDVelocity)
                done = True
                # Now take care of remaining parameters
                densityMapConstraints.pop("translation")
                densityMapConstraints.pop("velocity")
                for k, v in densityMapConstraints.items():
                    self._assignmentFns[k](obj, v, self._densityGenerationFns[k](v))
                break
            if not done:
                return False
        elif self._isParticulate(obj):
            # Place the center of the particle system -- but don't put anything in the scene yet
            gPDOrientation = self._densityGenerationFns["rotation"](densityMapConstraints["rotation"])
            orientation = samplePD(gPDOrientation)
            if not gPDOrientation:
                return False
            #gPDTranslation = [[1.0, [0.0, 0.0, 0.5]]]
            gPDTranslation = self._densityGenerationFns["translation"](obj, densityMapConstraints["translation"], orientation)
            densityMapConstraints.pop("translation")
            densityMapConstraints.pop("rotation")
            if not self._sampleAndValidateObject(obj.getMeshPath(modifier=self._space.volumePathModifier()), self._space.origin(), obj, collisionManager, gPDTranslation, orientation, attempts=self._poseSamplingAttempts, addToScene=False):
                return False
            particleNum = samplePD(self._getParticleNumPD(densityMapConstraints["particle_num"]))
            if "particles" in obj._parameters: 
                particleNum = len(obj._parameters["particles"])
            if "particles" not in obj._parameters:
                obj._parameters["particles"] = []
            densityMapConstraints.pop("particle_num")
            gPDTranslation = self._densityGenerationFns["particle_region_position"](densityMapConstraints["particle_region_position"], obj)
            gPDVelocity = self._densityGenerationFns["velocity"](densityMapConstraints["velocity"], obj)
            gPDOrientation = [[1.0, self._space.identityRotation()]]
            densityMapConstraints.pop("particle_region_position")
            densityMapConstraints.pop("velocity")
            j = 0
            for k in list(range(particleNum)):
                if len(obj._parameters["particles"]) <= j:
                    obj._parameters["particles"].append({})
                obj._parameters["particles"][j]["mesh"] = obj.getMeshPath(modifier="")
                if not self._sampleAndValidateObject(obj.getMeshPath(modifier=self._space.volumePathModifier()), self._space.origin(), obj._parameters["particles"][j], collisionManager, gPDTranslation, self._space.identityRotation(), attempts=self._poseSamplingAttempts, addToScene=True):
                    continue
                self._assignVelocity(obj._parameters["particles"][j], [True], gPDVelocity)
                for kk, v in densityMapConstraints.items():
                    if kk in obj._parameters:
                        obj._parameters["particles"][j][kk] = obj._parameters[kk]
                    self._assignmentFns[kk](obj._parameters["particles"][j], v, self._densityGenerationFns[kk](v))
                j = j + 1
            if (0 < len(obj._parameters["particles"])) and ("tx" not in obj._parameters["particles"][-1]):
                obj._parameters["particles"] = obj._parameters["particles"][:-1]
        return True

    # Just to not get into infinite loops, every object is explicit for the empty simulator ...
    def isExplicitObject(self, obj):
        return True
    def isExplicitSchema(self, schema):
        if "ParameterizedSchema" in schema._meta_type:
            return self.isExplicitObject(schema)
        if self.isExplicatableSchema(schema):
            for k,v in schema._roles.items():
                if not hasattr(v, "_meta_type"):
                    continue
                if not ((("RoleDefiningSchema" in v._meta_type) and self.isExplicitSchema(v)) or (("ParameterizedSchema" in v._meta_type) and self.isExplicitObject(v))):
                    return False
            return True
        return False
    def isExplicatableSchema(self, schema):
        return True

