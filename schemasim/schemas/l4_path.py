import os
import sys

import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l2_geometric_primitive_relations as gpr
import schemasim.schemas.l3_primitive_movement as pm
import schemasim.schemas.l3_location as location

class Path(st.RoleDefiningSchema):
    def __init__(self, source=None, destination=None):
        super().__init__()
        self._type = "Path"
        self._meta_type.append("Path")
        self._roles = {"source": source, "destination": destination}
    def _interpretPoint(self, role, frame, sim):
        if (role not in self._roles) or (not isinstance(self._roles[role], st.Schema)):
            return None, None
        s = self._roles[role]
        if "ParameterizedSchema" in s._meta_type:
            name = s.getId()
            return sim.translationVector(frame[name]), name
        elif "GeometricPrimitive" in s._meta_type:
            return s.getPoint(sim, frameData=frame), None
        # TODO: add branch for generating points from Location schemas
        return None, None
    def _pathAtFrame(self, frameNum, frame, sim, parameterizedSchemas, disabledObjects):
        space = sim.space()
        source, sourceName = self._interpretPoint("source", frame, sim)
        destination, destinationName = self._interpretPoint("destination", frame, sim)
        ignoreList = [sourceName, destinationName] + disabledObjects
        collisionManager = space.makeCollisionManager()
        maximumRadius = 0.0
        for name, data in frame.items():
            findSep = name.rfind(":")
            nameK = name
            if -1 != findSep:
                nameK = name[findSep]
            if (nameK in parameterizedSchemas) and (name not in ignoreList):
                mesh = space.loadVolume(parameterizedSchemas[nameK].getMeshPath(modifier=space.volumePathModifier()))
                t = sim.translationVector(data)
                pose = space.poseFromTR(t, sim.rotationRepresentation(data))
                collisionManager.add_object(name, mesh, pose)
                newRadius = 0.5*space.boundaryBoxDiameter(space.volumeBounds(mesh)) + space.vectorNorm(t)
                if maximumRadius < newRadius:
                    maximumRadius = newRadius
        if not source:
            source = space.vectorAboveOrigin(maximumRadius)
        if not destination:
            destination = space.vectorAboveOrigin(maximumRadius)
        path = space.planPath(source, destination, collisionManager, maximumRadius*1.01)
        return path
    def _pathCost(self, path, space):
        return 0.0
    def evaluateTimeline(self, frameData, simulator, parameterizedSchemas={}, disabledObjects=[]):
        space = simulator.space()
        cost = 0.0
        frameCount = len(frameData)
        for frameNum, frame in enumerate(frameData):
            crCost = self._pathCost(self._pathAtFrame(frameNum, frame, simulator, parameterizedSchemas, disabledObjects), space)
            if not crCost:
                cost = frameCount*1.0
                break
            else:
                cost = cost + crCost
        judgement = True
        if 0.1*frameCount < cost:
            judgement = False
        return judgement, cost

class PathAbsence(Path):
    def __init__(self, source=None, destination=None):
        super().__init__(source=source, destination=destination)
        self._type = "PathAbsence"
        self._meta_type.append("PathAbsence")
    def _pathCost(self, path, space):
        if path:
            return None
        return 0.0

class PathExistence(Path):
    def __init__(self, source=None, destination=None):
        super().__init__(source=source, destination=destination)
        self._type = "PathExistence"
        self._meta_type.append("PathExistence")
    def _pathCost(self, path, space):
        if not path:
            return None
        return 0.0

