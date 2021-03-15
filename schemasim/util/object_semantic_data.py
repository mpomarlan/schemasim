import os
import sys

import ast

def queryObjectSemanticData(meshPath, entry):
    if os.path.isfile(meshPath):
        sem = ast.literal_eval(open(meshPath).read())
        if entry in sem:
            return sem[entry]
    return None

