import schemasim.scene_generator as sg
import schemasim.simulators.blender_simulator as bs

import schemasim.objects.example_objects as eo
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l2_geometric_primitive_relations as gpr
import schemasim.schemas.l11_functional_control as fc

bsim = bs.BlenderSimulator()

# A list of objects in the scene. It's useful to define this first so that schematic relations are easier to define later.
pot = eo.Pot()
popcorn = eo.Popcorn()
board = eo.BalsaBoard()

# It helps to have an object to build the scene around. To create this, add some constraints on this object
# that involve only it and the world
aa = gpr.AxisAlignment(a=gp.WorldVerticalDirection(),b=gp.UprightDirection(obj=pot))


cov = fc.Coverage(coveree=pot, coverer=board)
cont = fc.Containment(container=pot, containee=popcorn)

# The order in which schemas are mentioned here is not that important, but it is important that all objects
# and all relevant relations between them are put in
results = sg.interpretScene([aa, pot, cont, popcorn, cov, board], bsim)

# Of course, other things would be interesting to do with results, e.g. learning rules of the form
# (qualitative description of scene) -> (qualitative description of behavior)
# or assigning blame/credit for observed behavior to objects in the scene


