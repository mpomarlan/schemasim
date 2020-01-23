import os
import sys

import random
import numpy as np

import math

import schemasim.simulators.physics_simulator_3D as phys_simulator_3D
import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l2_geometric_primitive_relations as gpr
#import schemasim.schemas.l3_primitive_movement as pm
#import schemasim.schemas.l3_location as location
#import schemasim.schemas.l4_path as path
#import schemasim.schemas.l10_expectation as expectation

import schemasim.objects.example_objects as eo

class BlenderSimulator(phys_simulator_3D.PhysicsSimulator3D):
    def __init__(self):
        super().__init__()
        return
    def getPathEnvironmentVariable(self):
        return "BLENDER_PATH"
    def typeName(self):
        return "BlenderSimulator"

    def _getMeshPD(self, constraints):
        return [[1.0, "~/Documents/AffTest/meshes/cube/cube.dae"]]
    def _getMassPD(self, constraints):
        return [[1.0, 1.0]]
    def _getRestitutionPD(self, constraints):
        return [[1.0, 0.3]]
    def _getFrictionPD(self, constraints):
        return [[1.0, 0.75]]
    def _getLinDmpPD(self, constraints):
        return [[1.0, 0.1]]
    def _getAngDmpPD(self, constraints):
        return [[1.0, 0.4]]

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
        retq = retq + "import json\n"
        retq = retq + "import ast\n\n"
        retq = retq + "def leetHAXX(err):\n"
        retq = retq + "    return ast.literal_eval(err.args[0][err.args[0].find('not found in ') + len('not found in '):])\n"
        retq = retq + "bpy.context.scene.render.fps = 24\n"
        retq = retq + "bpy.context.scene.render.fps_base = 1\n"
        retq = retq + "bpy.ops.rigidbody.world_add()\n"
        retq = retq + "bpy.context.scene.rigidbody_world.steps_per_second = 100\n"
        retq = retq + "bpy.context.scene.rigidbody_world.solver_iterations = 100\n"
        retq = retq + "bpy.context.scene.rigidbody_world.use_split_impulse = True\n"
        retq = retq + "scene = bpy.data.scenes['Scene']\n"
        retq = retq + "bpy.ops.object.select_all(action='SELECT')\n"
        retq = retq + "bpy.ops.object.delete(use_global=False)\n\n"
        retq = retq + "if 'lamp_add' in dir(bpy.ops.object):"
        retq = retq + "    bpy.ops.object.lamp_add(type='SUN', radius=1.31, location=(21.5562, 0, 28.3477))\n"
        retq = retq + "elif 'light_add' in dir(bpy.ops.object):"
        retq = retq + "    bpy.ops.object.light_add(type='SUN', radius=1.31, location=(21.5562, 0, 28.3477))\n"
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
            retq = retq + "vts = ('Standard')\n"
            retq = retq + "try:\n"
            retq = retq + "    bpy.context.scene.view_settings.view_transform = 'Bruh y u heff to make zis hard bruh'\n"
            retq = retq + "except TypeError as err:\n"
            retq = retq + "    vts = leetHAXX(err)\n"
            retq = retq + "if 'sRGB OETF' in vts:\n"
            retq = retq + "    bpy.context.scene.view_settings.view_transform = 'sRGB OETF'\n"
            retq = retq + "elif 'Standard' in vts:\n"
            retq = retq + "    bpy.context.scene.view_settings.view_transform = 'Standard'\n"
            retq = retq + "lks = ('Standard - High Contrast')\n"
            retq = retq + "try:\n"
            retq = retq + "    bpy.context.scene.view_settings.look = 'Bruh y u heff to make zis hard bruh'\n"
            retq = retq + "except TypeError as err:\n"
            retq = retq + "    lks = leetHAXX(err)\n"
            retq = retq + "if 'High Contrast' in lks:\n"
            retq = retq + "    bpy.context.scene.view_settings.look = 'High Contrast'\n"
            retq = retq + "elif 'Standard - High Contrast' in lks:\n"
            retq = retq + "    bpy.context.scene.view_settings.look = 'Standard - High Contrast'\n"
            retq = retq + "csns = [x.name for x in list(bpy.context.scene.sequencer_colorspace_settings.bl_rna.properties['name'].enum_items)]\n"
            retq = retq + "if 'sRGB OETF' in csns:\n"
            retq = retq + "    bpy.context.scene.sequencer_colorspace_settings.name = 'sRGB OETF'\n"
            retq = retq + "elif 'sRGB' in csns:\n"
            retq = retq + "    bpy.context.scene.sequencer_colorspace_settings.name = 'sRGB'\n"
        retq = retq + "scene.camera = bpy.data.objects['Camera']\n"
        retq = retq + ("bpy.ops.wm.collada_import(filepath='%s')\n" % eo.Floor().getMeshPath(modifier=""))
        retq = retq + "bpy.context.object.rotation_mode = 'QUATERNION'\n"
        retq = retq + ("bpy.context.active_object.name = 'Floor'\n")
        retq = retq + "bpy.ops.rigidbody.object_add()\n"
        retq = retq + "bpy.context.object.rigid_body.type = 'PASSIVE'\n"
        retq = retq + "bpy.context.object.rigid_body.collision_shape = 'MESH'\n"
        retq = retq + "bpy.context.object.rigid_body.mesh_source = 'BASE'\n"
        retq = retq + "bpy.context.object.rigid_body.collision_margin = 0.005\n"
        retq = retq + "bpy.context.object.rigid_body.restitution = 0.8\n"
        retq = retq + "bpy.ops.object.modifier_add(type='COLLISION')\n"
        retq = retq + "if 'select' in dir(bpy.context.active_object):"
        retq = retq + "    bpy.context.active_object.select = False\n\n"
        retq = retq + "elif 'select_set' in dir(bpy.context.active_object):"
        retq = retq + "    bpy.context.active_object.select_set(False)\n\n"
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
                    retq = retq + ("bpy.context.object.rigid_body.collision_margin = 0.005\n")
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
                    retq = retq + ("bpy.context.object.rigid_body.collision_margin = 0.005\n")
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
                        retq = retq + ("bpy.context.object.rigid_body.collision_margin = 0.005\n")
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
                            x = trajectories[pname][2]["tx"]
                            y = trajectories[pname][2]["ty"]
                            z = trajectories[pname][2]["tz"]
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
        retq = retq + "            frame_log[o.name] = {'tx':o.matrix_world.translation[0], 'ty':o.matrix_world.translation[1], 'tz':o.matrix_world.translation[2], 'rw':o.matrix_world.to_quaternion()[0], 'rx':o.matrix_world.to_quaternion()[1], 'ry':o.matrix_world.to_quaternion()[2], 'rz':o.matrix_world.to_quaternion()[3]}\n"
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

