import schemasim.scene_generator as sg
import schemasim.simulators.blender_simulator as bs

import schemasim.objects.example_objects as eo
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l2_geometric_primitive_relations as gpr
import schemasim.schemas.l11_functional_control as fc

bsim = bs.BlenderSimulator()

# A list of objects in the scene. It's useful to define this first so that schematic relations are easier to define later.
# !!!!!! NOTE: For this test to work, the mesh argument MUST be given a CORRECT ABSOLUTE PATH to the dae mesh. Adjust as needed when running it yourself.
pot = eo.MiscellaneousRigidObject(name="Pot", object_type="Pot", mesh="~/Documents/ownbranches/schemasim/schemasim/meshes/pot/pot.dae", mass=4.0, restitution=0.3, friction=0.75, linear_damping=0.4, angular_damping=0.5)
popcorn = eo.Popcorn()

# It helps to have an object to build the scene around. To create this, add some constraints on this object
# that involve only it and the world
aa = gpr.AxisAlignment(a=gp.WorldVerticalDirection(),b=gp.UprightDirection(obj=pot))

cont = fc.Containment(container=pot, containee=popcorn)

# The order in which schemas are mentioned here is not that important, but it is important that all objects
# and all relevant relations between them are put in
results = sg.interpretScene([aa, pot, cont, popcorn], bsim, simulate_counterfactuals=True, nframes=300)

print(results)

# Of course, other things would be interesting to do with results, e.g. learning rules of the form
# (qualitative description of scene) -> (qualitative description of behavior)
# or assigning blame/credit for observed behavior to objects in the scene

