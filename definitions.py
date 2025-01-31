#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@Introduce : Contains project constants
@File      : definitions.py
@Project   : BrickScanner
@Time      : 06.03.22 14:52
@Author    : flowmeadow
"""

import os

# paths
ROOT_DIR  = os.path.dirname(os.path.abspath(__file__))
IMG_DIR   = os.path.join(ROOT_DIR, '../images')
DATA_DIR  = os.path.join(ROOT_DIR, '../data')
# BRICK_DIR = '/home/user/ldraw/parts'
# STL_DIR   = '/home/florian/ldraw/stl'
BRICK_DIR = os.path.join(ROOT_DIR, '../ldraw/parts')
STL_DIR   = os.path.join(ROOT_DIR, '../stl')

# create dirs ...
for dir in [IMG_DIR, DATA_DIR, STL_DIR]:
    os.makedirs(dir, exist_ok=True)
