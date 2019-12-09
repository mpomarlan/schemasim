import math

from math import sin, cos

import numpy as np

from schemasim.util.geometry import fibonacci_sphere, transformVector

import random

def normalizePD(pd):
    if not pd:
        return None
    csum = 0.0
    for c in pd:
        csum = csum + c[0]
    if 0.000001 > csum:
        csum = 1.0
    for c in pd:
        c[0] = c[0]/csum
    return pd
def samplePD(probabilityDensity):
    if not probabilityDensity:
        return None
    phi = random.uniform(0.0, 1.0)
    for c in probabilityDensity:
        phi = phi - c[0]
        if phi <= 0:
            return c[1]
    if 0 < len(probabilityDensity):
        return probabilityDensity[-1][1]
    return None

def uniformQuaternionRPD(dimsteps=10):
    pd = []
    axis = fibonacci_sphere(samples=8*(dimsteps*dimsteps))
    angles = list(np.arange(-np.pi, np.pi, (np.pi/dimsteps)))
    angles = angles[:dimsteps] + angles[dimsteps+1:]
    pd.append([1.0, [0.0, 0.0, 0.0, 1.0]])
    for ax in axis:
        for ang in angles:
            c = cos(ang/2.0)
            s = sin(ang/2)
            if 0 > s:
                s = -s
                c = -c
            pd.append([1.0, [s*ax[0], s*ax[1], s*ax[2], c]])
    return pd

def uniformBoxRPD(dims, translation=[0,0,0], rotation=[0,0,0,1], resolution=0.1):
    pd = []
    xs = list(np.arange(dims[0][0], dims[0][1]*1.01, resolution))
    ys = list(np.arange(dims[1][0], dims[1][1]*1.01, resolution))
    zs = list(np.arange(dims[2][0], dims[2][1]*1.01, resolution))
    for x in xs:
        for y in ys:
            for z in zs:
                pd.append([1.0, transformVector([x, y, z], translation, rotation)])
    return pd
 
