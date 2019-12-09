# schemasim
Naive physics reasoning with simulation: go from a qualitative description of a scene to a qualitative description of observed behaviors, and assign responsibility for the observed behaviors to objects in the scene.

Dependencies:

* python 3.4 or newer
* blender, version 2.78c (newer may work also)
* trimesh (sudo pip3 install trimesh)
* rtree (sudo apt-get install python3-rtree)

Optionally,

* python-fcl

Also, assumes that an environment variable is set which holds the path to the simulator. For using the Blender simulator, this variable is BLENDER_PATH.

