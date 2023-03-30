#!/usr/bin/env python

import numpy as np
from glpg_flowmeadow.transformations.methods import rot_mat
from definitions import *
from lib.helper.lego_bricks import load_stl
from scripts.sim_recon_setup import prepare_mesh

import open3d as o3d

from typing import Tuple


class Brick:
    def __init__(self):
        # defaults
        self.step = 0.04
        self.n_p, self.r = 8, 1.5
        self.z_offset = 0.2

        self.brick_ids = ["3039", "3010"]

    def __getitem__(self, brick_id: str) -> Tuple[o3d.geometry.TriangleMesh, o3d.geometry.PointCloud, float, int, float, float, str]:
        """
        :return: [mesh, pointcloud, step, n_p, r, z_offset, brick_id]
        """

        if brick_id in self.brick_ids:
            getattr(self, f"b{brick_id}")()
            return self.mesh, self.pc_gt, self.step, self.n_p, self.r, self.z_offset, brick_id
        else:
            raise Exception(f"Brick {brick_id} not found")

    def all_bricks(self):
        for brick_id in self.brick_ids:
            yield self[brick_id]

    def b3039(self):
        mesh = load_stl("3039")
        mesh = prepare_mesh(mesh)
        mesh.scale(1.6, np.zeros(3))
        mesh.rotate(rot_mat((0., 0., 1.), 90), mesh.get_center())
        mesh.translate(np.array([1.6, -2.4, 0.2]))

        self.pc_gt = mesh.sample_points_uniformly(100_000)
        self.mesh = mesh

    def b3010(self):
        mesh = load_stl("3010")
        mesh = prepare_mesh(mesh)
        mesh.scale(1.6, np.zeros(3))
        # mesh.rotate(rot_mat((0., 0., 1.), 100), mesh.get_center())
        mesh.rotate(rot_mat((0., 0., 1.), 135), mesh.get_center())
        mesh.rotate(rot_mat((0., 1., 0.), -40), mesh.get_center())
        mesh.rotate(rot_mat((0., 0., 1.), 25), mesh.get_center())
        # mesh.translate(np.array([0.5, -2.5, 0.1]))
        mesh.translate(np.array([1.6, -2.5, 0.1]))

        self.pc_gt = mesh.sample_points_uniformly(100_000)
        self.mesh = mesh

        self.z_offset = 0.5

if __name__ == "__main__":
    print(Brick()["3010"])

    for b in Brick().all_bricks():
        print(b)


# Florian

# brick_id = "3039"
# # step = 0.17 * 3 / 10
# step = 0.04
# mesh = load_stl(brick_id)
# mesh = prepare_mesh(mesh)
# mesh.scale(1.6, np.zeros(3))
# mesh.rotate(rot_mat((0., 0., 1.), 90), mesh.get_center())
# mesh.translate(np.array([1.6, -2.4, 0.2]))
# n_p, r = 8, 1.5
# z_offset = 0.2

# white
# brick_id = "6152"
# step = 0.17 * 3 / 10
# mesh = load_stl(brick_id)
# mesh = prepare_mesh(mesh)
# mesh.scale(1.6, np.zeros(3))
# mesh.rotate(rot_mat((0., 0., 1.), 80), mesh.get_center())
# mesh.translate(np.array([0.5, -0.5, 0.1]))
# n_p, r = 24, 2.0
# z_offset = 0.

# green
# brick_id = "3001"
# step = 0.17 * 3 / 10
# mesh = load_stl(brick_id)
# mesh = prepare_mesh(mesh)
# mesh.scale(1.6, np.zeros(3))
# mesh.rotate(rot_mat((0., 0., 1.), 75), mesh.get_center())
# mesh.translate(np.array([1.0, -1.8, 0.1]))
#
# mesh.compute_triangle_normals()
# n_p, r = 8, 1.5
# z_offset = 0.4

# orange
# brick_id = "3298"
# step = 0.17 * 3 / 10
# mesh = load_stl(brick_id)
# mesh = prepare_mesh(mesh)
# mesh.scale(1.6, np.zeros(3))
# mesh.rotate(rot_mat((0., 0., 1.), 75), mesh.get_center())
# mesh.translate(np.array([1.6, -2.4, 0.3]))
# n_p, r = 8, 1.5
# z_offset = 0.

# blue
# brick_id = "3298"
# step = 0.17 * 3 / 10
# mesh = load_stl(brick_id)
# mesh = prepare_mesh(mesh)
# mesh.scale(1.6, np.zeros(3))
# mesh.rotate(rot_mat((0., 0., 1.), 75), mesh.get_center())
# mesh.translate(np.array([1.6, -2.4, 0.3]))
# n_p, r = 8, 1.5
# z_offset = 0.2

# 3010
# brick_id = "3010"
# step = 0.17 * 3 / 10
# mesh = load_stl(brick_id)
# mesh = prepare_mesh(mesh)
# mesh.scale(1.6, np.zeros(3))
# # mesh.rotate(rot_mat((0., 0., 1.), 100), mesh.get_center())
# mesh.rotate(rot_mat((0., 0., 1.), 135), mesh.get_center())
# mesh.rotate(rot_mat((0., 1., 0.), -40), mesh.get_center())
# mesh.rotate(rot_mat((0., 0., 1.), 25), mesh.get_center())
# mesh.translate(np.array([0.5, -2.5, 0.1]))
# mesh.translate(np.array([1.6, -2.5, 0.1]))
# n_p, r = 8, 1.5
# z_offset = 0.5
