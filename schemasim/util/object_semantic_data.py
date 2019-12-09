import os
import sys

import ast

def queryObjectSemanticData(meshPath, entry):
    semPath = meshPath[:meshPath.rfind(".")+1] + "sem"
    if os.path.isfile(semPath):
        sem = ast.literal_eval(open(semPath).read())
        if entry in sem:
            return sem[entry]
    return None

