import os
import sys

class SimulatorResponse:
    def __init__(self, sceneSetup):
        self.result = None
        self._sceneSetup = sceneSetup

class Simulator:
    # A base class for simulators, this simulates nothing.
    def __init__(self):
        return
    def getPathEnvironmentVariable(self):
        return "<n/a: trivial sim has no path>"
    def getPath(self):
        return os.environ.get(self.getPathEnvironmentVariable())
    def typeName(self):
        return "TrivialSimulator"
    # The empty simulator is compatible with any schema-- it won't do anything anyway.
    def compatiblePhysics(self, schemas):
        return True
    # Just to not get into infinite loops, every object and goal description is explicit for the empty simulator ...
    def isExplicitObject(self, obj):
        return True
    def isExplicitSchema(self, schema):
        return True
    def isExplicatableSchema(self, schema):
        return True
    def isExplicitGoal(self, goal):
        return True
    def parameterizeObject(self, obj, constraintSchemas):
        return True
    ######### TODO: these need refactoring or deletion
    # ... it just does nothing and returns an empty result in all cases.
    def __call__(self, explicitSceneSchemas):
        return SimulatorResponse(sceneSetup=explicitSceneSchemas)
    def getExplications(explicitSchemas, parSchemas, simExplicatable):
        snet = SchemaNet(explicitSchemas)
        snet.addSchemas(parSchemas)
        snet.addSchemas(simExplicatable)
        return [snet]
    def setupScene(self, schemas):
        return True

