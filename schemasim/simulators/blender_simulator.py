import os
import sys

import random
import numpy as np

import math

import trimesh
import schemasim.simulators.simulator as simulator
import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l2_geometric_primitive_relations as gpr
#import schemasim.schemas.l3_primitive_movement as pm
#import schemasim.schemas.l3_location as location
#import schemasim.schemas.l4_path as path
#import schemasim.schemas.l10_expectation as expectation

import schemasim.objects.example_objects as eo

from schemasim.util.geometry import centroid, poseFromTQ, transformVector, fibonacci_sphere, distanceFromInterior, outerAreaFromSurface
from schemasim.util.probability_density import normalizePD, samplePD, uniformQuaternionRPD, uniformBoxRPD

def filterByAxisAlignment(quaternionRPD, targetAxis, movingAxis, strictness=0.005):
    for c in quaternionRPD:
        movedAxis = transformVector(movingAxis, [0,0,0], c[1])
        angle = math.acos((targetAxis[0]*movedAxis[0] + targetAxis[1]*movedAxis[1] + targetAxis[2]*movedAxis[2]))
        c[0] = c[0]*math.exp(-angle/strictness)
    return quaternionRPD

def filterByAxisOrthogonality(quaternionRPD, targetAxis, movingAxis, strictness=0.005):
    for c in quaternionRPD:
        movedAxis = transformVector(movingAxis, [0,0,0], c[1])
        angle = math.acos((targetAxis[0]*movedAxis[0] + targetAxis[1]*movedAxis[1] + targetAxis[2]*movedAxis[2]))
        c[0] = c[0]*math.exp(-math.fabs(angle-math.pi/2)/strictness)
    return quaternionRPD

def filterByMovingPointInVolume(translationRPD, targetVolume, movingPoint, strictness=0.005):
    if not targetVolume:
        return None
    targetVolumeRayIntersector = trimesh.ray.ray_triangle.RayMeshIntersector(targetVolume)
    for c in translationRPD:
        movedPoint = transformVector(movingPoint, c[1], [0,0,0,1])
        cost = distanceFromInterior(movedPoint, targetVolume, targetVolumeRayIntersector)
        c[0] = c[0]/math.exp(cost/strictness)
        centroid = list(targetVolume.centroid)
        dx = movedPoint[0]-centroid[0]
        dy = movedPoint[1]-centroid[1]
        dz = movedPoint[2]-centroid[2]
        d = (dx*dx + dy*dy + dz*dz)
        c[0] = c[0]/math.pow((1.0 + d), 10)
        if 0.001 < cost:
            c[0] = c[0]/math.exp(cost/strictness)
    return translationRPD
def filterByTargetPointInVolume(translationRPD, targetPoint, movingVolume, strictness=0.005):
    if not movingVolume:
        return None
    last = [0,0,0]
    for c in translationRPD:
        current = [c[1][0] - last[0], c[1][1] - last[1], c[1][2] - last[2]]
        movingVolume.apply_translation(current)
        movingVolumeRayIntersector = trimesh.ray.ray_triangle.RayMeshIntersector(movingVolume)
        cost = distanceFromInterior(targetPoint, movingVolume, movingVolumeRayIntersector)
        c[0] = c[0]/math.exp(cost/strictness)
        last = c[1]
    movingVolume.apply_translation([-last[0], -last[1], -last[2]])
    return translationRPD
def filterBySurfaceContainment(translationRPD, targetSurface, movingSurface, strictness=0.005):
    if not movingSurface:
        return None
    targetCenter = centroid(targetSurface)
    for c in translationRPD:
        movedSurface = []
        for e in movingSurface:
            movedSurface.append(transformVector(e, c[1], [0,0,0,1]))
        movedCenter = centroid(movedSurface)
        cost = outerAreaFromSurface(movedSurface, targetSurface)
        dx = movedCenter[0] - targetCenter[0]
        dy = movedCenter[1] - targetCenter[1]
        dz = movedCenter[2] - targetCenter[2]
        d = (dx*dx + dy*dy + dz*dz)
        c[0] = c[0]/math.exp(cost/strictness)
        c[0] = c[0]/(math.pow((1.0 + d), 4))
    return translationRPD

class BlenderSimulator(simulator.Simulator):
    def getPathEnvironmentVariable(self):
        return "BLENDER_PATH"
    def typeName(self):
        return "BlenderSimulator"
    def compatiblePhysics(self, schemas):
        for s in schemas:
            if "ParameterizedSchema" in s._meta_type:
                if ("physics_type" not in s._parameters) or (s._parameters["physics_type"] not in ["rigid_body", "soft_body", "impulse_particles"]):
                    return False
        return True
    def getMeshPD(self, constraints):
        return [[1.0, "~/Documents/AffTest/meshes/cube/cube.dae"]]
    def getPhysicsTypePD(self, constraints):
        return [[1.0, "rigid_body"]]
    def getParticleNumPD(self, constraints):
        return [[1.0, 1]]
    def getMassPD(self, constraints):
        return [[1.0, 1.0]]
    def getRestitutionPD(self, constraints):
        return [[1.0, 0.3]]
    def getFrictionPD(self, constraints):
        return [[1.0, 0.75]]
    def getLinDmpPD(self, constraints):
        return [[1.0, 0.1]]
    def getAngDmpPD(self, constraints):
        return [[1.0, 0.4]]

    def getParticleRegionPD(self, constraints, obj):
        pd = [[1.0, [0,0,0]]]
        if ("radius" in obj._parameters):
            r = obj._parameters["radius"]
            pd = uniformBoxRPD([[-r, r], [-r, r], [-r, r]], [0.0,0.0,0.0], [0.0,0.0,0.0,1.0], resolution=0.04)
            for c in pd:
                d = math.sqrt(c[1][0]*c[1][0] + c[1][1]*c[1][1] + c[1][2]*c[1][2])
                if r < d:
                    c[0] = 0.0
            for c in pd:
                c[1] = [c[1][0] + obj._parameters["tx"], c[1][1] + obj._parameters["ty"], c[1][2] + obj._parameters["tz"]]
        for c in constraints:
            if "PointInVolume" == c._type:
                volume = c.getTargetVolume(self)
                pd = filterByMovingPointInVolume(pd, volume, [0.0, 0.0, 0.0], strictness=0.005)
            if "SurfaceContainment" == c._type:
                movingSurface = [[0.0, 0.0, 0.0]]
                targetSurface = c.getTargetSurface(self)
                pd = filterBySurfaceContainment(pd, targetSurface, movingSurface, strictness=0.005)
        return normalizePD(pd)
    def getParticleVelocityPD(self, constraints, obj):
        pd = [[1.0, [0,0,0]]]
        if (not constraints) and ("v_min" in obj._parameters):
            vmin = obj._parameters["v_min"]
            vmax = obj._parameters["v_max"]
            vdir = obj._parameters["v_direction"]
            vbias = obj._parameters["v_bias"]
            pd = []
            for speed in list(np.arange(vmin, vmax, (vmax-vmin)/15.0)):
                for direction in fibonacci_sphere(samples=100, only_positive_quadrant=False):
                    pd.append([1.0, [speed*direction[0], speed*direction[1], speed*direction[2]]])
            if 0.01 < vbias:
                for c in pd:
                    a = 0.5*(vdir[0]*c[1][0] + vdir[1]*c[1][1] + vdir[2]*c[1][2]) + 0.5
                    s = math.pow(a, vbias)
                    c[0] = s*c[0]
        return normalizePD(pd)
    def getTranslationPD(self, obj, constraints, quaternion):
        if not constraints:
            volume = obj.getVolume()
            if volume:
                z = -list(volume.bounds)[0][2]
            else:
                z = 0.0
            return [[1.0, [0.0,0.0,z]]]
        pd = []
        for c in constraints:
            dims = None
            if "PointInVolume" == c._type:
                dims, translation, rotation = c.getTargetVolumeBounds(self)
                if c.getMovingVolume(self):
                    rotation = quaternion
                boxPD = uniformBoxRPD(dims, translation=translation, rotation=rotation, resolution=0.15)
                if c.getMovingVolume(self):
                    targetPoint = c.getTargetPoint(self)
                    for j in list(range(len(boxPD))):
                        boxPD[j] = [boxPD[j][0], [targetPoint[0] - boxPD[j][1][0], targetPoint[1] - boxPD[j][1][1], targetPoint[2] - boxPD[j][1][2]]]
            if "SurfaceContainment" == c._type:
                dims, translation, rotation = c.getTargetSurfaceBounds(self)
                center = centroid(c.getMovingSurface(self))
                center = transformVector(center, [0,0,0], quaternion)
                boxPD = uniformBoxRPD(dims, translation=translation, rotation=rotation, resolution=0.4)
                for j in list(range(len(boxPD))):
                    boxPD[j] = [boxPD[j][0], [boxPD[j][1][0] - center[0], boxPD[j][1][1] - center[1], boxPD[j][1][2] - center[2]]]
            if dims:
                pd = pd + boxPD
        strictness = 0.005
        for c in constraints:
            if "PointInVolume" == c._type:
                movingPoint = c.getMovingPoint(self)
                targetPoint = c.getTargetPoint(self)
                if movingPoint:
                    movingPoint = transformVector(movingPoint, [0,0,0], quaternion)
                    volume = c.getTargetVolume(self)
                    pd = filterByMovingPointInVolume(pd, volume, movingPoint, strictness=strictness)
                elif targetPoint:
                    volume = c.getMovingVolume(self)
                    volume = volume.apply_transform(poseFromTQ([0,0,0], quaternion))
                    pd = filterByTargetPointInVolume(pd, targetPoint, volume, strictness=strictness)
            if "SurfaceContainment" == c._type:
                movingSurface = c.getMovingSurface(self)
                targetSurface = c.getTargetSurface(self)
                for j in list(range(len(movingSurface))):
                    movingSurface[j] = transformVector(movingSurface[j], [0,0,0], quaternion)
                pd = filterBySurfaceContainment(pd, targetSurface, movingSurface, strictness=strictness)
        return normalizePD(pd)
    def getRotationPD(self, constraints):
        constraintsInterp = []
        for c in constraints:
            if isinstance(c, gpr.AxisAlignment) or isinstance(c, gpr.AxisCounterAlignment) or isinstance(c, gpr.AxisOrthogonality):
                constraintsInterp.append(c)
            elif isinstance(c, gpr.SurfaceContainment):
                constraintsInterp.append(gpr.AxisCounterAlignment(a=gp.SurfaceNormal(surface=c._roles["container_surface"]), b=gp.SurfaceNormal(surface=c._roles["containee_surface"])))
        if not constraintsInterp:
            return [[1.0, [0.0,0.0,0.0,1]]]
        pd = uniformQuaternionRPD()
        strictness = 0.005
        if 1 < len(constraintsInterp):
            strictness = strictness*len(constraintsInterp)
        for c in constraintsInterp:
            if c._type in ["AxisAlignment", "AxisCounterAlignment", "AxisOrthogonality", "SurfaceContainment"]:
                targetAxis = c.getTargetAxis(self)
                movingAxis = c.getMovingAxis(self)
                if isinstance(c, gpr.AxisAlignment) or isinstance(c, gpr.AxisCounterAlignment):
                    pd = filterByAxisAlignment(pd, targetAxis, movingAxis, strictness=strictness)
                elif isinstance(c, gpr.AxisOrthogonality):
                    pd = filterByAxisOrthogonality(pd, targetAxis, movingAxis, strictness=strictness)
        return normalizePD(pd)
    def getVelocityPD(self, constraints):
        return [[1.0, [0,0,0]]]
    def getAngVelPD(self, constraints):
        return [[1.0, [0,0,0]]]

    def _assignParameter(self, obj, constraints, pd, parameter):
        if hasattr(obj, "_parameters"):
            params = obj._parameters
        else:
            params = obj
        if constraints or (parameter not in params):
            params[parameter] = samplePD(pd)
    def assignMesh(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "mesh")
    def assignPhysicsType(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "physics_type")
    def assignMass(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "mass")
    def assignRestitution(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "restitution")
    def assignFriction(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "friction")
    def assignFriction(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "friction")
    def assignLinDamp(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "linear_damping")
    def assignAngDamp(self, obj, constraints, pd):
        self._assignParameter(obj, constraints, pd, "angular_damping")
    def assignVelocity(self, obj, constraints, pd):
        if not pd:
            pd = [[1.0, [0.0, 0.0, 0.0]]]
        vel = samplePD(pd)
        self._assignParameter(obj, constraints, [[1.0, vel[0]]], "vx")
        self._assignParameter(obj, constraints, [[1.0, vel[1]]], "vy")
        self._assignParameter(obj, constraints, [[1.0, vel[2]]], "vz")
    def assignAngVel(self, obj, constraints, pd):
        if not pd:
            pd = [[1.0, [0.0, 0.0, 0.0]]]
        vel = samplePD(pd)
        self._assignParameter(obj, constraints, [[1.0, vel[0]]], "wx")
        self._assignParameter(obj, constraints, [[1.0, vel[1]]], "wy")
        self._assignParameter(obj, constraints, [[1.0, vel[2]]], "wz")

    def sampleAndValidateObject(self,meshPath,cx,cy,cz,obj,collisionManager,gPDTranslation,gPDQuaternion,attempts=20,addToScene=True):
        if (not gPDTranslation) or (not gPDQuaternion) or (not meshPath):
            return False
        k = 0
        mesh = trimesh.load(meshPath)
        while (k < attempts):
            translation = samplePD(gPDTranslation)
            translation[0] = translation[0] + cx
            translation[1] = translation[1] + cy
            translation[2] = translation[2] + cz
            rotation = samplePD(gPDQuaternion)
            pose = poseFromTQ(translation, rotation)
            collision= collisionManager.in_collision_single(mesh, pose)
            if not collision:
                name = "Object_%d" % (len(collisionManager._objs))
                if "name" in obj._parameters:
                    name = obj._parameters["name"]
                elif "id" in obj._parameters:
                    name = obj._parameters["id"]
                if addToScene:
                    collisionManager.add_object(name, mesh, pose)
                obj._parameters["tx"] = translation[0] - cx
                obj._parameters["ty"] = translation[1] - cy
                obj._parameters["tz"] = translation[2] - cz
                obj._parameters["rx"] = rotation[0]
                obj._parameters["ry"] = rotation[1]
                obj._parameters["rz"] = rotation[2]
                obj._parameters["rw"] = rotation[3]
                return True
            k = k + 1
        return False
    def sampleAndValidateParticle(self,meshPath,cx,cy,cz,particle,collisionManager,gPDTranslation,gPDQuaternion,attempts=20):
        if (not gPDTranslation) or (not gPDQuaternion) or (not meshPath):
            return False
        k = 0
        mesh = trimesh.load(meshPath)
        while (k < attempts):
            translation = samplePD(gPDTranslation)
            translation[0] = translation[0] + cx
            translation[1] = translation[1] + cy
            translation[2] = translation[2] + cz
            rotation = samplePD(gPDQuaternion)
            pose = poseFromTQ(translation, rotation)
            if not collisionManager.in_collision_single(mesh, pose):
                name = "Particle_%d" % (len(collisionManager._objs))
                collisionManager.add_object(name, mesh, pose)
                particle["tx"] = translation[0] - cx
                particle["ty"] = translation[1] - cy
                particle["tz"] = translation[2] - cz
                particle["rx"] = rotation[0]
                particle["ry"] = rotation[1]
                particle["rz"] = rotation[2]
                particle["rw"] = rotation[3]
                return True
            k = k + 1
        return False
    def parameterizeObject(self, obj, constraintSchemas, collisionManager):
        # TODO: make it so that particle positions also get checked by main object translation constraints
        # These can be adjusted later once the scene is initialized, to process expectations
        obj._parameters['has_collision'] = 1
        obj._parameters['is_kinematic'] = 0
        # Group constraints by the properties they target
        densityMapConstraints = {("px", "py", "pz"): [], ("pvx", "pvy", "pvz"): [], ("tx", "ty", "tz"): [], ("rx", "ry", "rz", "rw"): [], ("vx", "vy", "vz"): [], ("wx", "wy", "wz"): [], ("mesh",): [], ("physics_type",): [], ("particle_num",): [], ("mass",): [], ("restitution",): [], ("friction",): [], ("linear_damping",): [], ("angular_damping",): []}
        densityGenerationFns = {("px", "py", "pz"): self.getParticleRegionPD, ("pvx", "pvy", "pvz"): self.getParticleVelocityPD, ("tx", "ty", "tz"): self.getTranslationPD, ("rx", "ry", "rz", "rw"): self.getRotationPD, ("vx", "vy", "vz"): self.getVelocityPD, ("wx", "wy", "wz"): self.getAngVelPD, ("mesh",): self.getMeshPD, ("physics_type",): self.getPhysicsTypePD, ("particle_num",): self.getParticleNumPD, ("mass",): self.getMassPD, ("restitution",): self.getRestitutionPD, ("friction",): self.getFrictionPD, ("linear_damping",): self.getLinDmpPD, ("angular_damping",): self.getAngDmpPD}
        assignmentFns = {("vx", "vy", "vz"): self.assignVelocity, ("wx", "wy", "wz"): self.assignAngVel, ("mesh",): self.assignMesh, ("physics_type",): self.assignPhysicsType, ("mass",): self.assignMass, ("restitution",): self.assignRestitution, ("friction",): self.assignFriction, ("linear_damping",): self.assignLinDamp, ("angular_damping",): self.assignAngDamp}
        for s in constraintSchemas:
            if s._type in ["AxisAlignment", "AxisCounterAlignment", "AxisOrthogonality", "SurfaceContainment"]:
                densityMapConstraints[("rx", "ry", "rz", "rw")].append(s)
            if s._type in ["PointInVolume", "SurfaceContainment"]:
                densityMapConstraints[("tx", "ty", "tz")].append(s)
                densityMapConstraints[("px", "py", "pz")].append(s)
        # Begin by selecting a mesh ...
        self.assignMesh(obj, densityMapConstraints[("mesh",)], self.getMeshPD(densityMapConstraints[("mesh",)]))
        # ... and a physics type
        self.assignPhysicsType(obj, densityMapConstraints[("physics_type",)], self.getPhysicsTypePD(densityMapConstraints[("physics_type",)]))
        densityMapConstraints.pop(("mesh",))
        densityMapConstraints.pop(("physics_type",))
        # Assign other properties depending on physics_type
        if obj._parameters["physics_type"] in ["rigid_body", "soft_body"]:
            densityMapConstraints.pop(("particle_num",))
            densityMapConstraints.pop(("pvx", "pvy", "pvz"))
            densityMapConstraints.pop(("px", "py", "pz"))
            # Rotation and translation need special handling
            done = False
            gPDQuaternion = densityGenerationFns[("rx", "ry", "rz", "rw")](densityMapConstraints[("rx", "ry", "rz", "rw")])
            if not gPDQuaternion:
                return False
            densityMapConstraints.pop(("rx", "ry", "rz", "rw"))
            for attempt in list(range(10)):
                quaternion = samplePD(gPDQuaternion)
                gPDTranslation = densityGenerationFns[("tx", "ty", "tz")](obj, densityMapConstraints[("tx", "ty", "tz")], quaternion)
                if not self.sampleAndValidateObject(obj.getMeshPath(modifier=".stl"), 0.0, 0.0, 0.0, obj, collisionManager, gPDTranslation, [[1.0, quaternion]]):
                    continue
                done = True
                # Now take care of remaining parameters
                densityMapConstraints.pop(("tx", "ty", "tz"))
                for k, v in densityMapConstraints.items():
                    assignmentFns[k](obj, v, densityGenerationFns[k](v))
                break
            if not done:
                return False
        elif obj._parameters["physics_type"] in ["impulse_particles"]:
            # Place the center of the particle system -- but don't put anything in the scene yet
            gPDQuaternion = densityGenerationFns[("rx", "ry", "rz", "rw")](densityMapConstraints[("rx", "ry", "rz", "rw")])
            quaternion = samplePD(gPDQuaternion)
            if not gPDQuaternion:
                return False
            gPDTranslation = densityGenerationFns[("tx", "ty", "tz")](obj, densityMapConstraints[("tx", "ty", "tz")], quaternion)
            densityMapConstraints.pop(("tx", "ty", "tz"))
            densityMapConstraints.pop(("rx", "ry", "rz", "rw"))
            if not self.sampleAndValidateObject(obj.getMeshPath(modifier=".stl"), 0.0, 0.0, 0.0, obj, collisionManager, gPDTranslation, gPDQuaternion, addToScene=False):
                return False
            particleNum = 30
            if densityMapConstraints[("particle_num",)]:
                particleNum = samplePD(self.getParticleNumPD(densityMapConstraints[("particle_num",)]))
            elif "particles" in obj._parameters: 
                particleNum = len(obj._parameters["particles"])
            if "particles" not in obj._parameters:
                obj._parameters["particles"] = []
            densityMapConstraints.pop(("particle_num",))
            gPDTranslation = densityGenerationFns[("px", "py", "pz")](densityMapConstraints[("px", "py", "pz")], obj)
            gPDVelocity = densityGenerationFns[("pvx", "pvy", "pvz")](densityMapConstraints[("pvx", "pvy", "pvz")], obj)
            gPDQuaternion = [[1.0, [0,0,0,1]]]
            densityMapConstraints.pop(("px", "py", "pz"))
            densityMapConstraints.pop(("pvx", "pvy", "pvz"))
            for k in list(range(particleNum)):
                if len(obj._parameters["particles"]) <= k:
                    obj._parameters["particles"].append({})
                if not self.sampleAndValidateParticle(obj.getMeshPath(modifier=".stl"), 0.0, 0.0, 0.0, obj._parameters["particles"][k], collisionManager, gPDTranslation, gPDQuaternion):
                    continue
                velocity = samplePD(gPDVelocity)
                obj._parameters["particles"][k]["vx"] = velocity[0]
                obj._parameters["particles"][k]["vy"] = velocity[1]
                obj._parameters["particles"][k]["vz"] = velocity[2]
                for kk, v in densityMapConstraints.items():
                    if kk not in ["mesh", "mass"]:
                        assignmentFns[kk](obj._parameters["particles"][k], v, densityGenerationFns[kk](v))
                # TODO: clean up mass assignment for particles
                obj._parameters["particles"][k]["mesh"] = obj.getMeshPath(modifier="")
                obj._parameters["particles"][k]["mass"] = obj._parameters["mass"]
            validParticles = []
            for p in obj._parameters["particles"]:
                if "mesh" in p:
                    validParticles.append(p)
            obj._parameters["particles"] = validParticles
        return True
    def isExplicitObject(self, obj):
        # To initialize a rigid or soft object, blender must know:
        #   its centroid-in-world position and velocity, its rotation in world and angular velocity
        #   whether the object is affected by forces from the simulator
        #   whether the object has collision detection enabled
        #   what "physics type" the object has: rigid body, soft body, particle system
        #   an object name
        if obj._parameters["physics_type"] in ["rigid_body", "soft_body"]:
            if list(set(["tx", "ty", "tz", "rx", "ry", "rz", "rw", "vx", "vy", "vz", "wx", "wy", "wz", "is_kinematic", "has_collision", "physics_type", "name"]) - set(obj._parameters.keys())):
                return False
        # rigid_objects and soft_bodies also need a mesh to describe their shape, and mass, restitution, friction, linear_damping, angular_damping parameters parameters
        if obj._parameters["physics_type"] in ["rigid_body", "soft_body"]:
            if list(set(["friction", "restitution", "mesh", "mass", "linear_damping", "angular_damping"]).difference(set(obj._parameters.keys()))):
                return False
        # soft objects need stretch-, compress-, bend resistance, damping, plasticity parameters
        if "soft_body" == obj._parameters["physics_type"]:
            if set(["stretch_resistance", "compress_resistance", "bend_resistance", "damping", "plasticity"]).difference(set(obj._parameters.keys())):
                return False
        # impulse particles need "is_kinematic", "has_collision", "name", and a particles list, and each particle in the list must have 
        #   "tx", "ty", "tz", "vx", "vy", "vz", "friction", "restitution", "mesh", "mass", "linear_damping", "angular_damping"
        if "impulse_particles" == obj._parameters["physics_type"]:
            if ("particles" not in obj._parameters.keys()) or (not obj._parameters["particles"]):
                return False
            for p in obj._parameters["particles"]:
                if set(["tx", "ty", "tz", "vx", "vy", "vz", "friction", "restitution", "mesh", "mass", "linear_damping", "angular_damping"]).difference(p.keys()):
                    return False
        return True
    def isExplicitSchema(self, schema):
        if self.isExplicatableSchema(schema):
            for k,v in schema._roles.items():
                if not hasattr(v, "_meta_type"):
                    continue
                if not ((("RoleDefiningSchema" in v._meta_type) and self.isExplicitSchema(v)) or (("ParameterizedSchema" in v._meta_type) and self.isExplicitObject(v))):
                    return False
            return True
        return False
    def isExplicatableSchema(self, schema):
        # the following schemas do not require further theory to elucidate, as far as blender is concerned:
        if schema._type in ["Object", "ParticleSystem", "Var", "WorldVerticalDirection", "WorldRelativeTopSurface", "ObjectRelativeBottomSurface", "UprightDirection", "Centroid", "Interior", "CollisionEnabled", "CollisionDisabled", "SurfaceContainment", "PointInVolume", "AxisAlignment", "RelativeDepart", "RelativeStay", "RelativeStayLevel", "RelativeFall", "RelativeApproach", "Expectation", "PathAbsence", "PathExistence"]:
            return True
        return False
    def sceneScript(self, schemas, save_folder, blender_filename=None, log_filename=None, trajectories=None, render=False):
        dt = 1.0/24.0
        if not blender_filename:
            blender_filename = "animation.blend"
        if not log_filename:
            log_filename = "animation_log.txt"
        blender_outfile_path = os.path.join(save_folder, blender_filename)
        blender_logfile_path = os.path.join(save_folder, log_filename)
        retq = ""
        retq = retq + "import os\n"
        retq = retq + "import bpy\n"
        retq = retq + "import sys\n"
        retq = retq + "import json\n\n"
        retq = retq + "bpy.context.scene.render.fps = 24\n"
        retq = retq + "bpy.context.scene.render.fps_base = 1\n"
        retq = retq + "bpy.ops.rigidbody.world_add()\n"
        retq = retq + "bpy.context.scene.rigidbody_world.steps_per_second = 200\n"
        retq = retq + "bpy.context.scene.rigidbody_world.solver_iterations = 100\n"
        retq = retq + "bpy.context.scene.rigidbody_world.use_split_impulse = True\n"
        retq = retq + "scene = bpy.data.scenes['Scene']\n"
        retq = retq + "bpy.ops.object.select_all(action='SELECT')\n"
        retq = retq + "bpy.ops.object.delete(use_global=False)\n\n"
        retq = retq + "bpy.ops.object.lamp_add(type='SUN', radius=1.31, location=(21.5562, 0, 28.3477))\n"
        retq = retq + "bpy.context.object.rotation_mode = 'QUATERNION'\n"
        retq = retq + "bpy.context.object.rotation_quaternion[0] = 0.949\n"
        retq = retq + "bpy.context.object.rotation_quaternion[1] = 0\n"
        retq = retq + "bpy.context.object.rotation_quaternion[2] = 0.319\n"
        retq = retq + "bpy.context.object.rotation_quaternion[3] = 0\n"
        retq = retq + "bpy.context.object.data.energy = 1.5\n"
        retq = retq + "bpy.ops.object.camera_add(location=(12.6112, 0, 10.4034), rotation=(0, 0, 0))\n"
        retq = retq + "bpy.context.object.rotation_mode = 'QUATERNION'\n"
        retq = retq + "bpy.context.object.rotation_quaternion[0] = 0.623\n"
        retq = retq + "bpy.context.object.rotation_quaternion[1] = 0.335\n"
        retq = retq + "bpy.context.object.rotation_quaternion[2] = 0.335\n"
        retq = retq + "bpy.context.object.rotation_quaternion[3] = 0.623\n"
        retq = retq + "bpy.context.object.data.type = 'ORTHO'\n"
        retq = retq + "bpy.context.object.data.ortho_scale = 10\n"
        retq = retq + "bpy.context.object.data.sensor_width = 24.89\n"
        retq = retq + "bpy.context.object.data.sensor_height = 18.66\n"
        retq = retq + "bpy.context.object.data.sensor_fit = 'HORIZONTAL'\n"
        if render:
            retq = retq + "bpy.context.scene.display_settings.display_device = 'sRGB'\n"
            retq = retq + "bpy.context.scene.view_settings.view_transform = 'sRGB OETF'\n"
            retq = retq + "bpy.context.scene.view_settings.look = 'High Contrast'\n"
            retq = retq + "bpy.context.scene.sequencer_colorspace_settings.name = 'sRGB OETF'\n"
        retq = retq + "scene.camera = bpy.data.objects['Camera']\n"
        retq = retq + ("bpy.ops.wm.collada_import(filepath='%s')\n" % eo.Floor().getMeshPath(modifier=""))
        retq = retq + "bpy.context.object.rotation_mode = 'QUATERNION'\n"
        retq = retq + ("bpy.context.active_object.name = '%s'\n" % "Floor")
        retq = retq + "bpy.ops.rigidbody.object_add()\n"
        retq = retq + "bpy.context.object.rigid_body.type = 'PASSIVE'\n"
        retq = retq + "bpy.context.object.rigid_body.collision_shape = 'MESH'\n"
        retq = retq + "bpy.context.object.rigid_body.mesh_source = 'BASE'\n"
        retq = retq + "bpy.context.object.rigid_body.collision_margin = 0.0\n"
        retq = retq + "bpy.context.object.rigid_body.restitution = 0.8\n"
        retq = retq + "bpy.ops.object.modifier_add(type='COLLISION')\n"
        retq = retq + "bpy.context.active_object.select = False\n\n"
        retq = retq + "bpy.data.scenes['Scene'].frame_end = 250\n"
        retq = retq + "bpy.data.scenes['Scene'].rigidbody_world.point_cache.frame_end = 250\n"
        for o in schemas:
            if "ParameterizedSchema" not in o._meta_type:
                continue
            if "rigid_body" == o._parameters["physics_type"]:
                retq = retq + ("bpy.ops.wm.collada_import(filepath='%s')\n" % o.getMeshPath(modifier=""))
                retq = retq + ("bpy.context.active_object.name = '%s'\n" % o.getId())
                retq = retq + ("new_obj = bpy.data.objects['%s']\n" % o.getId())
                retq = retq + ("new_obj.location = (%f,%f,%f)\n" % (o._parameters["tx"], o._parameters["ty"], o._parameters["tz"]))
                retq = retq + "bpy.context.object.rotation_mode = 'QUATERNION'\n"
                retq = retq + ("new_obj.rotation_quaternion = (%f,%f,%f,%f)\n" % (o._parameters["rw"], o._parameters["rx"], o._parameters["ry"], o._parameters["rz"]))
                retq = retq + "new_obj.keyframe_insert(data_path='location', frame=1)\n"
                retq = retq + "new_obj.keyframe_insert(data_path='rotation_quaternion', frame=1)\n"
                if o._parameters["has_collision"]:
                    retq = retq + "bpy.ops.rigidbody.object_add()\n"
                    retq = retq + "bpy.context.object.rigid_body.type = 'ACTIVE'\n"
                    retq = retq + "bpy.context.object.rigid_body.enabled = True\n"
                    retq = retq + "bpy.context.object.rigid_body.kinematic = True\n"
                    retq = retq + "new_obj.keyframe_insert(data_path='rigid_body.kinematic', frame=1)\n"
                    retq = retq + ("bpy.context.object.rigid_body.mass = %f\n" % o._parameters["mass"])
                    retq = retq + ("bpy.context.object.rigid_body.collision_shape = 'MESH'\n")
                    retq = retq + ("bpy.context.object.rigid_body.mesh_source = 'BASE'\n")
                    retq = retq + ("bpy.context.object.rigid_body.collision_margin = 0.0\n")
                    retq = retq + ("bpy.context.object.rigid_body.friction = %f\n" % o._parameters["friction"])
                    retq = retq + ("bpy.context.object.rigid_body.restitution = %f\n" % o._parameters["restitution"])
                    retq = retq + ("bpy.context.object.rigid_body.linear_damping = %f\n" % o._parameters["linear_damping"])
                    retq = retq + ("bpy.context.object.rigid_body.angular_damping = %f\n" % o._parameters["angular_damping"])
                    retq = retq + "bpy.ops.object.modifier_add(type='COLLISION')\n"
            elif "soft_body" == o._parameters["physics_type"]:
                retq = retq + ("bpy.ops.wm.collada_import(filepath='%s')\n" % o.getMeshPath(modifier=""))
                retq = retq + ("bpy.context.active_object.name = '%s'\n" % o.getId())
                retq = retq + ("new_obj = bpy.data.objects['%s']\n" % o.getId())
                retq = retq + ("new_obj.location = (%f,%f,%f)\n" % (o._parameters["tx"], o._parameters["ty"], o._parameters["tz"]))
                retq = retq + "bpy.context.object.rotation_mode = 'QUATERNION'\n"
                retq = retq + ("new_obj.rotation_quaternion = (%f,%f,%f,%f)\n" % (o._parameters["rw"], o._parameters["rx"], o._parameters["ry"], o._parameters["rz"]))
                retq = retq + "new_obj.keyframe_insert(data_path='location', frame=1)\n"
                retq = retq + "new_obj.keyframe_insert(data_path='rotation_quaternion', frame=1)\n"
                if o._parameters["has_collision"]:
                    retq = retq + "bpy.ops.rigidbody.object_add()\n"
                    retq = retq + "bpy.context.object.rigid_body.type = 'ACTIVE'\n"
                    retq = retq + "bpy.context.object.rigid_body.kinematic = True\n"
                    retq = retq + "bpy.context.object.rigid_body.enabled = True\n"
                    retq = retq + "new_obj.keyframe_insert(data_path='rigid_body.kinematic', frame=1)\n"
                    retq = retq + ("bpy.context.object.rigid_body.collision_shape = 'MESH'\n")
                    retq = retq + ("bpy.context.object.rigid_body.mesh_source = 'DEFORM'\n")
                    retq = retq + ("bpy.context.object.rigid_body.use_deform = True\n")
                    retq = retq + ("bpy.context.object.rigid_body.collision_margin = 0.0\n")
                    retq = retq + ("bpy.context.object.rigid_body.friction = %f\n" % o._parameters["friction"])
                    retq = retq + ("bpy.context.object.rigid_body.restitution = %f\n" % o._parameters["restitution"])
                    retq = retq + ("bpy.context.object.rigid_body.linear_damping = %f\n" % o._parameters["linear_damping"])
                    retq = retq + ("bpy.context.object.rigid_body.angular_damping = %f\n" % o._parameters["angular_damping"])
                    retq = retq + "bpy.ops.object.modifier_add(type='COLLISION')\n"
                    retq = retq + "bpy.ops.object.modifier_add(type='SOFT_BODY')\n"
                    retq = retq + "bpy.context.object.modifiers['Softbody'].point_cache.frame_end = 250\n"
                    retq = retq + ("bpy.context.object.modifiers['Softbody'].settings.friction = %f\n" % o._parameters["friction"])
                    retq = retq + ("bpy.context.object.modifiers['Softbody'].settings.mass = %f\n" % o._parameters["mass"])
                    retq = retq + ("bpy.context.object.modifiers['Softbody'].settings.use_goal = False\n")
                    retq = retq + ("bpy.context.object.modifiers['Softbody'].settings.use_edges = True\n")
                    retq = retq + ("bpy.context.object.modifiers['Softbody'].settings.pull = %f\n" % o._parameters["stretch_resistance"])
                    retq = retq + ("bpy.context.object.modifiers['Softbody'].settings.push = %f\n" % o._parameters["compress_resistance"])
                    retq = retq + ("bpy.context.object.modifiers['Softbody'].settings.damping = %f\n" % o._parameters["damping"])
                    retq = retq + ("bpy.context.object.modifiers['Softbody'].settings.plastic = %f\n" % o._parameters["plasticity"])
                    retq = retq + ("bpy.context.object.modifiers['Softbody'].settings.bend = %f\n" % o._parameters["bend_resistance"])
            elif "fluid_particles" == o._parameters["physics_type"]:
                continue
            elif "impulse_particles" == o._parameters["physics_type"]:
                k = 0
                for p in o._parameters["particles"]:
                    retq = retq + ("bpy.ops.wm.collada_import(filepath='%s')\n" % p["mesh"])
                    pname = o._parameters["name"] + ":" + str(k)
                    retq = retq + ("bpy.context.active_object.name = '%s'\n" % pname)
                    retq = retq + ("new_obj = bpy.data.objects['%s']\n" % pname)
                    retq = retq + ("new_obj.location = (%f,%f,%f)\n" % (p["tx"], p["ty"], p["tz"]))
                    retq = retq + "bpy.context.object.rotation_mode = 'QUATERNION'\n"
                    retq = retq + "new_obj.keyframe_insert(data_path='location', frame=1)\n"
                    if o._parameters["has_collision"]:
                        retq = retq + "bpy.ops.rigidbody.object_add()\n"
                        retq = retq + "bpy.context.object.rigid_body.type = 'ACTIVE'\n"
                        retq = retq + "bpy.context.object.rigid_body.enabled = True\n"
                        retq = retq + "bpy.context.object.rigid_body.kinematic = True\n"
                        retq = retq + "new_obj.keyframe_insert(data_path='rigid_body.kinematic', frame=1)\n"
                        retq = retq + ("bpy.context.object.rigid_body.mass = %f\n" % p["mass"])
                        retq = retq + ("bpy.context.object.rigid_body.collision_shape = 'MESH'\n")
                        retq = retq + ("bpy.context.object.rigid_body.mesh_source = 'BASE'\n")
                        retq = retq + ("bpy.context.object.rigid_body.collision_margin = 0.0\n")
                        retq = retq + ("bpy.context.object.rigid_body.friction = %f\n" % p["friction"])
                        retq = retq + ("bpy.context.object.rigid_body.restitution = %f\n" % p["restitution"])
                        retq = retq + ("bpy.context.object.rigid_body.linear_damping = %f\n" % p["linear_damping"])
                        retq = retq + ("bpy.context.object.rigid_body.angular_damping = %f\n" % p["angular_damping"])
                        retq = retq + "bpy.ops.object.modifier_add(type='COLLISION')\n"
                    k = k + 1
        retq = retq + "\n"
        retq = retq + "scene.frame_current = 2\n"
        for o in schemas:
            if "ParameterizedSchema" not in o._meta_type:
                continue
            if o._parameters["physics_type"] in ["rigid_body", "soft_body"]:
                retq = retq + ("obj = bpy.data.objects['%s']\n" % o._parameters["name"])
                has_trajectory = trajectories and (o._parameters["name"] in trajectories)
                if o._parameters["has_collision"] and (not o._parameters["is_kinematic"]):
                    dqw = 0.5*(-o._parameters["wx"]*o._parameters["rx"]-o._parameters["wy"]*o._parameters["ry"]-o._parameters["wz"]*o._parameters["rz"])
                    dqx = 0.5*(o._parameters["wx"]*o._parameters["rw"]-o._parameters["wz"]*o._parameters["ry"]+o._parameters["wy"]*o._parameters["rz"])
                    dqy = 0.5*(o._parameters["wy"]*o._parameters["rw"]+o._parameters["wz"]*o._parameters["rx"]-o._parameters["wx"]*o._parameters["rz"])
                    dqz = 0.5*(o._parameters["wz"]*o._parameters["rw"]-o._parameters["wy"]*o._parameters["rx"]+o._parameters["wx"]*o._parameters["ry"])
                    retq = retq + ("obj.location = (%f,%f,%f)\n" % (o._parameters["tx"]+dt*o._parameters["vx"], o._parameters["ty"]+dt*o._parameters["vy"], o._parameters["tz"]+dt*o._parameters["vz"]))
                    retq = retq + ("obj.rotation_quaternion = (%f,%f,%f,%f)\n" % (o._parameters["rw"]+dt*dqw, o._parameters["rx"]+dt*dqx, o._parameters["ry"]+dt*dqy, o._parameters["rz"]+dt*dqz))
                    retq = retq + "obj.keyframe_insert(data_path='location', frame=2)\n"
                    retq = retq + "obj.keyframe_insert(data_path='rotation_quaternion', frame=2)\n"
                elif has_trajectory and (2 in trajectories[o._parameters["name"]]):
                    x = trajectories[o._parameters["name"]][2]["tx"]
                    y = trajectories[o._parameters["name"]][2]["ty"]
                    z = trajectories[o._parameters["name"]][2]["tz"]
                    qw = trajectories[o._parameters["name"]][2]["rw"]
                    qx = trajectories[o._parameters["name"]][2]["rx"]
                    qy = trajectories[o._parameters["name"]][2]["ry"]
                    qz = trajectories[o._parameters["name"]][2]["rz"]
                    retq = retq + ("obj.location = (%f,%f,%f)\n" % (x,y,z))
                    retq = retq + ("obj.rotation_quaternion = (%f,%f,%f,%f)\n" % (qw,qx,qy,qz))
                    retq = retq + "obj.keyframe_insert(data_path='location', frame=2)\n"
                    retq = retq + "obj.keyframe_insert(data_path='rotation_quaternion', frame=2)\n"
            elif "impulse_particles" == o._parameters["physics_type"]:
                    k = 0
                    for p in o._parameters["particles"]:
                        if not p:
                            continue
                        pname = o._parameters["name"] + ":" + str(k)
                        retq = retq + ("obj = bpy.data.objects['%s']\n" % pname)
                        has_trajectory = trajectories and (pname in trajectories)
                        if o._parameters["has_collision"] and (not o._parameters["is_kinematic"]):
                            retq = retq + ("obj.location = (%f,%f,%f)\n" % (p["tx"]+dt*p["vx"], p["ty"]+dt*p["vy"], p["tz"]+dt*p["vz"]))
                            retq = retq + "obj.keyframe_insert(data_path='location', frame=2)\n"
                        elif has_trajectory and (2 in trajectories[pname]):
                            x = trajectories[o._parameters["name"]][2]["tx"]
                            y = trajectories[o._parameters["name"]][2]["ty"]
                            z = trajectories[o._parameters["name"]][2]["tz"]
                            retq = retq + ("obj.location = (%f,%f,%f)\n" % (x,y,z))
                            retq = retq + "obj.keyframe_insert(data_path='location', frame=2)\n"
                        k = k + 1
        retq = retq + "\n"
        retq = retq + "scene.frame_current = 3\n"
        for o in schemas:
            if "ParameterizedSchema" not in o._meta_type:
                continue
            if o._parameters["has_collision"] and (o._parameters["physics_type"] in ["rigid_body", "soft_body"]) and (not o._parameters["is_kinematic"]):
                retq = retq + ("obj = bpy.data.objects['%s']\n" % o._parameters["name"])
                retq = retq + "obj.rigid_body.kinematic = False\n"
                retq = retq + "obj.keyframe_insert(data_path='rigid_body.kinematic', frame=3)\n"
            elif o._parameters["has_collision"] and (o._parameters["physics_type"] in ["impulse_particles"]) and (not o._parameters["is_kinematic"]):
                k = 0
                for p in o._parameters["particles"]:
                    pname = o._parameters["name"] + ":" + str(k)
                    retq = retq + ("obj = bpy.data.objects['%s']\n" % pname)
                    retq = retq + "obj.rigid_body.kinematic = False\n"
                    retq = retq + "obj.keyframe_insert(data_path='rigid_body.kinematic', frame=3)\n"
                    k = k + 1
        retq = retq + "\n"
        for f in range(3, 251):
            set_frame = False
            for o in schemas:
                if "ParameterizedSchema" not in o._meta_type:
                    continue
                if o._parameters["physics_type"] in ["rigid_body", "soft_body"]:
                    if trajectories and (o._parameters["name"] in trajectories) and (f in trajectories[o._parameters["name"]]) and ((not o._parameters["has_collision"]) or (o._parameters["is_kinematic"])):
                        if not set_frame:
                            retq = retq + ("scene.frame_current = %d\n" % f)
                            set_frame = True
                        retq = retq + ("obj = bpy.data.objects['%s']\n" % o._parameters["name"])
                        x = trajectories[o._parameters["name"]][f]["tx"]
                        y = trajectories[o._parameters["name"]][f]["ty"]
                        z = trajectories[o._parameters["name"]][f]["tz"]
                        qw = trajectories[o._parameters["name"]][f]["rw"]
                        qx = trajectories[o._parameters["name"]][f]["rx"]
                        qy = trajectories[o._parameters["name"]][f]["ry"]
                        qz = trajectories[o._parameters["name"]][f]["rz"]
                        retq = retq + ("obj.location = (%f,%f,%f)\n" % (x,y,z))
                        retq = retq + ("obj.rotation_quaternion = (%f,%f,%f,%f)\n" % (qw,qx,qy,qz))
                        retq = retq + ("obj.keyframe_insert(data_path='location', frame=%d)\n" % f)
                        retq = retq + ("obj.keyframe_insert(data_path='rotation_quaternion', frame=%d)\n" % f)
                elif o._parameters["physics_type"] in ["impulse_particles"]:
                    k = 0
                    for p in o._parameters["particles"]:
                        pname = o._parameters["name"] + ":" + str(k)
                        if trajectories and (pname in trajectories) and (f in trajectories[pname]) and ((not o._parameters["has_collision"]) or (o._parameters["is_kinematic"])):
                            if not set_frame:
                                retq = retq + ("scene.frame_current = %d\n" % f)
                                set_frame = True
                            retq = retq + ("obj = bpy.data.objects['%s']\n" % pname)
                            x = trajectories[pname][f]["tx"]
                            y = trajectories[pname][f]["ty"]
                            z = trajectories[pname][f]["tz"]
                            qw = trajectories[pname][f]["rw"]
                            qx = trajectories[pname][f]["rx"]
                            qy = trajectories[pname][f]["ry"]
                            qz = trajectories[pname][f]["rz"]
                            retq = retq + ("obj.location = (%f,%f,%f)\n" % (x,y,z))
                            retq = retq + ("obj.rotation_quaternion = (%f,%f,%f,%f)\n" % (qw,qx,qy,qz))
                            retq = retq + ("obj.keyframe_insert(data_path='location', frame=%d)\n" % f)
                            retq = retq + ("obj.keyframe_insert(data_path='rotation_quaternion', frame=%d)\n" % f)
                        k = k+1
        retq = retq + "\n"
        retq = retq + "bpy.ops.ptcache.bake_all(bake=True)\n\n"
        retq = retq + ("if not os.path.isdir('%s'):\n" % save_folder)
        retq = retq + ("    os.mkdir('%s')\n\n" % save_folder)
        retq = retq + ("if os.path.isfile('%s'):\n" % blender_outfile_path)
        retq = retq + ("    os.remove('%s')\n" % blender_outfile_path)
        retq = retq + ("bpy.ops.wm.save_as_mainfile(filepath='%s')\n\n" % blender_outfile_path)
        retq = retq + ("with open('%s', 'w') as outfile:\n" % blender_logfile_path)
        retq = retq + "    for frame in range(1, scene.frame_end):\n"
        retq = retq + "        scene.frame_set(frame)\n"
        retq = retq + "        frame_log = {}\n" 
        retq = retq + "        for o in bpy.data.objects:\n"
        retq = retq + "            if 'Floor' == o.name:\n"
        retq = retq + "                continue\n"
        retq = retq + "            frame_log[o.name] = {'x':o.matrix_world.translation[0], 'y':o.matrix_world.translation[1], 'z':o.matrix_world.translation[2], 'rw':o.matrix_world.to_quaternion()[0], 'rx':o.matrix_world.to_quaternion()[1], 'ry':o.matrix_world.to_quaternion()[2], 'rz':o.matrix_world.to_quaternion()[3]}\n"
        retq = retq + "        outfile.write('%s\\n' % json.dumps(frame_log))\n"
        retq = retq + "bpy.context.scene.render.resolution_x = 800\n"
        retq = retq + "bpy.context.scene.render.resolution_y = 600\n"
        retq = retq + "bpy.context.scene.render.resolution_percentage = 100\n"
        retq = retq + "scene.render.image_settings.file_format = 'FFMPEG'\n"
        retq = retq + "bpy.context.scene.render.ffmpeg.format = 'MPEG2'\n"
        if "/" != save_folder[-1]:
            save_folder = save_folder + "/"
        retq = retq + ("bpy.context.scene.render.filepath = '%s'\n" % save_folder)
        if render:
            retq = retq + "bpy.ops.render.render(animation=True)\n"
        retq = retq + "sys.exit()\n"
        return retq


