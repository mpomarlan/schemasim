#!/usr/bin/env python

from setuptools import setup


setup(
    name='schemasim',
    author='Mihai Pomarlan',
    license='MIT',
    install_requires=['trimesh',
                      'rtree',
                      'scipy',
                      'python-fcl'],
)
