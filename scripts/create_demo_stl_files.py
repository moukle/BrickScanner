#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@Introduce : Converts selected Ldraw files from BRICK_DIR to STL file format and stores them in STL_DIR
@File      : create_stl_files.py
@Project   : BrickScanner
@Time      : 21.07.22 20:30
@Author    : flowmeadow
"""
import os
import sys
import pickle
import numpy as np

sys.path.append(os.getcwd())  # required to run script from console

from definitions import *
from lib.helper.lego_bricks import convert_to_stl

if __name__ == "__main__":
    """
    Converts selected Ldraw files from BRICK_DIR to STL file format and stores them in STL_DIR.
    LDView needs to be installed. Only tested in Linux
    """

    # selected stl files for demo comparision
    stl_files = [ "3049.stl", "3049d.stl", "48.stl", "3049c.stl", "98387.stl", "3049b.stl", "2545.stl", "84625.stl", "2545p01.stl", "3049a.stl", "43745.stl", "43745p01.stl", "2791a.stl", "2607.stl", "56.stl", "2790.stl", "3137.stl", "3660.stl", "2757.stl", "3137a.stl", "3660p02.stl", "266bc01.stl", "4505a.stl", "3039ps5.stl", "66727.stl", "18976.stl", "3039pcg.stl", "3039p73.stl", "3039pz1.stl", "3039p67.stl", "3039pcc.stl", "3039p72.stl", "3039pci.stl", "3039p33.stl", "6227.stl", "3039p68.stl", "3039p58.stl", "3039pch.stl", "3039p70.stl", "3039ps2.stl", "3039p50.stl", "3039p08.stl", "3660p03.stl", "3039pt1.stl", "63341.stl", "3039ph2.stl", "3039p07.stl", "3039pcf.stl", "3039p34.stl", "3660p01.stl"]
    files = [f"{stl[:-4]}.dat" for stl in stl_files]
    files = [f for f in files if os.path.exists(BRICK_DIR + "/" + f)]

    for idx, file in enumerate(files):
        print(f"Processing file {file} ({idx + 1}|{len(files)})")
        file_base = file[:-4]
        out_file = f"{file_base}.stl"
        if out_file in os.listdir(STL_DIR):
            print("File already converted!")
            continue
        convert_to_stl(file_base, f"{STL_DIR}/{out_file}")
