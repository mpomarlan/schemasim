import os
import sys

import schemasim.schemas.l0_schema_templates as st

class Floor(st.ParameterizedSchema):
    def __init__(self, name="Floor"):
        super().__init__()
        self._meta_type.append("Floor")
        self._parameters["type"] = "Floor"
        self._parameters["name"] = "Floor"
        self._parameters["mesh"] = "floor/floor.dae"
        self._parameters["physics_type"] = "rigid_body"
        self._parameters["mass"] = 1.0
        self._parameters["restitution"] = 0.3
        self._parameters["friction"] = 0.7
        self._parameters["linear_damping"] = 0.3
        self._parameters["angular_damping"] = 0.3
        self._parameters["tx"] = 0.0
        self._parameters["ty"] = 0.0
        self._parameters["tz"] = 0.0
        self._parameters["rx"] = 0.0
        self._parameters["ry"] = 0.0
        self._parameters["rz"] = 0.0
        self._parameters["rw"] = 1.0
        self._parameters["vx"] = 0.0
        self._parameters["vy"] = 0.0
        self._parameters["vz"] = 0.0
        self._parameters["wx"] = 0.0
        self._parameters["wy"] = 0.0
        self._parameters["wz"] = 0.0
        self._parameters["has_collision"] = 1
        self._parameters["is_kinematic"] = 1

class Cube(st.ParameterizedSchema):
    def __init__(self, name="Cube", mass=1.0, restitution=0.3, friction=0.7, linear_damping=0.4, angular_damping=0.5):
        super().__init__()
        self._meta_type.append("Cube")
        self._parameters["type"] = "Cube"
        self._parameters["name"] = name
        self._parameters["mesh"] = "cube/cube.dae"
        self._parameters["physics_type"] = "rigid_body"
        self._parameters["mass"] = mass
        self._parameters["restitution"] = restitution
        self._parameters["friction"] = friction
        self._parameters["linear_damping"] = linear_damping
        self._parameters["angular_damping"] = angular_damping
        self._parameters["has_collision"] = 1
        self._parameters["is_kinematic"] = 0

class Pot(st.ParameterizedSchema):
    def __init__(self, name="Pot", mass=4.0, restitution=0.3, friction=0.75, linear_damping=0.4, angular_damping=0.5):
        super().__init__()
        self._meta_type.append("Pot")
        self._parameters["type"] = "Pot"
        self._parameters["name"] = name
        self._parameters["mesh"] = "pot/pot.dae"
        self._parameters["physics_type"] = "rigid_body"
        self._parameters["mass"] = mass
        self._parameters["restitution"] = restitution
        self._parameters["friction"] = friction
        self._parameters["linear_damping"] = linear_damping
        self._parameters["angular_damping"] = angular_damping
        self._parameters["has_collision"] = 1
        self._parameters["is_kinematic"] = 0

class BalsaBoard(st.ParameterizedSchema):
    def __init__(self, name="BalsaBoard", mass=0.02, restitution=0.1, friction=0.75, linear_damping=0.4, angular_damping=0.5):
        super().__init__()
        self._meta_type.append("BalsaBoard")
        self._parameters["type"] = "BalsaBoard"
        self._parameters["name"] = name
        self._parameters["mesh"] = "balsa_board/balsa_board.dae"
        self._parameters["physics_type"] = "rigid_body"
        self._parameters["mass"] = mass
        self._parameters["restitution"] = restitution
        self._parameters["friction"] = friction
        self._parameters["linear_damping"] = linear_damping
        self._parameters["angular_damping"] = angular_damping
        self._parameters["has_collision"] = 1
        self._parameters["is_kinematic"] = 0

class Cup(st.ParameterizedSchema):
    def __init__(self, name="Cup", mass=0.4, restitution=0.1, friction=0.75, linear_damping=0.4, angular_damping=0.5):
        super().__init__()
        self._meta_type.append("Cup")
        self._parameters["type"] = "Cup"
        self._parameters["name"] = name
        self._parameters["mesh"] = "cup/cup.dae"
        self._parameters["physics_type"] = "rigid_body"
        self._parameters["mass"] = mass
        self._parameters["restitution"] = restitution
        self._parameters["friction"] = friction
        self._parameters["linear_damping"] = linear_damping
        self._parameters["angular_damping"] = angular_damping
        self._parameters["has_collision"] = 1
        self._parameters["is_kinematic"] = 0

class Lid(st.ParameterizedSchema):
    def __init__(self, name="Lid", mass=1.5, restitution=0.3, friction=0.75, linear_damping=0.4, angular_damping=0.5):
        super().__init__()
        self._meta_type.append("Lid")
        self._parameters["type"] = "Lid"
        self._parameters["name"] = name
        self._parameters["mesh"] = "lid/lid.dae"
        self._parameters["physics_type"] = "rigid_body"
        self._parameters["mass"] = mass
        self._parameters["restitution"] = restitution
        self._parameters["friction"] = friction
        self._parameters["linear_damping"] = linear_damping
        self._parameters["angular_damping"] = angular_damping
        self._parameters["has_collision"] = 1
        self._parameters["is_kinematic"] = 0

class Popcorn(st.ParticleSystem):
    def __init__(self, name="Popcorn", mass=0.003, restitution=0.4, friction=0.75, linear_damping=0.4, angular_damping=0.5, radius=0.5, v_min=3.0, v_max=9.0, v_direction=[0.0,0.0,1.0], v_bias=2.0):
        super().__init__()
        self._meta_type.append("Popcorn")
        self._parameters["type"] = "Popcorn"
        self._parameters["name"] = name
        self._parameters["mesh"] = "popcorn/popcorn.dae"
        self._parameters["physics_type"] = "impulse_particles"
        self._parameters["mass"] = mass
        self._parameters["restitution"] = restitution
        self._parameters["friction"] = friction
        self._parameters["linear_damping"] = linear_damping
        self._parameters["angular_damping"] = angular_damping
        self._parameters["has_collision"] = 1
        self._parameters["is_kinematic"] = 0
        self._parameters["radius"] = radius
        self._parameters["v_min"] = v_min
        self._parameters["v_max"] = v_max
        self._parameters["v_direction"] = v_direction
        self._parameters["v_bias"] = v_bias


class MiscellaneousRigidObject(st.ParameterizedSchema):
    def __init__(self, name="RigidObject", object_type="MiscellaneousRigidObject", mesh="", mass=1.5, restitution=0.3, friction=0.75, linear_damping=0.4, angular_damping=0.5):
        super().__init__()
        self._meta_type.append(object_type)
        self._parameters["type"] = object_type
        self._parameters["name"] = name
        self._parameters["mesh"] = mesh
        self._parameters["physics_type"] = "rigid_body"
        self._parameters["mass"] = mass
        self._parameters["restitution"] = restitution
        self._parameters["friction"] = friction
        self._parameters["linear_damping"] = linear_damping
        self._parameters["angular_damping"] = angular_damping
        self._parameters["has_collision"] = 1
        self._parameters["is_kinematic"] = 0
    def getMeshPath(self, modifier=None):
        if "mesh" not in self._parameters:
            return None
        path = self._parameters["mesh"]
        if isinstance(modifier, str) and ("" != modifier):
            path = path[:path.rfind(".")] + modifier
        if not os.path.isfile(path):
            return None
        return path

