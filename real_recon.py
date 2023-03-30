#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@Introduce : TODO
@File      : real_recon.py
@Project   : BrickScanner
@Time      : 26.08.22 21:06
@Author    : flowmeadow
"""
import numpy as np

from definitions import *
from lib.recon.reconstruction import reconstruct_point_cloud
import open3d as o3d

from meshes_gt import Brick


def icp(pc_source, pc_target):
    threshold = 1.0
    trans_init = np.eye(4)
    reg_p2l = o3d.pipelines.registration.registration_icp(
        pc_source,
        pc_target,
        threshold,
        trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    )
    T_icp = reg_p2l.transformation
    T_icp = np.linalg.inv(T_icp)
    return T_icp


def main():
    img_dir  = f"{IMG_DIR}/3039"
    data_dir = f"{DATA_DIR}/real_setup/setup_D"

    _, _, step, n_p, r, z_offset, _ = Brick()["3039"]

    recon_kwargs = dict(
        gap_window=0,
        y_extension=1.0,
        step=step,
    )


    pc = reconstruct_point_cloud(data_dir, img_dir, **recon_kwargs)
    # o3d.visualization.draw_geometries([pc])
    o3d.io.write_point_cloud(f"{img_dir}/tmp.pcd", pc)

    pc = o3d.io.read_point_cloud(f"{img_dir}/tmp.pcd")
    pc: o3d.geometry.PointCloud
    pc, _ = pc.remove_radius_outlier(nb_points=n_p, radius=r * step)

    points = np.array(pc.points)
    points = points[points[:, 2] > z_offset]
    pc.points = o3d.utility.Vector3dVector(points)

    # compare with all defined bricks
    min: tuple[str, int] = ("", np.inf)
    for B in Brick().all_bricks():
        mesh, pc_gt, *_, brick_id = B
        pc.points = o3d.utility.Vector3dVector(points)

        T = icp(pc, pc_gt)
        mesh.transform(T)
        pc_gt.transform(T)

        # post processing
        center = mesh.get_center()
        for g in [mesh, pc_gt, pc]:
            g.translate(-center)
            # g.rotate(rot_mat((1., 0., 0.), -30), center)

        # metrics
        mean_dist = np.mean(pc.compute_point_cloud_distance(pc_gt))
        print(f"Mean distance to {brick_id}: {mean_dist}")

        if mean_dist <= min[1]:
            min = (brick_id, mean_dist)

        draw = False
        if draw:
            mesh = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
            o3d.visualization.draw_geometries([mesh, pc], mesh_show_wireframe=True, left=400)
    
    # display best result
    print(f"Best result with brick {min[0]} ({min[1]})")

    mesh, pc_gt, *_ = Brick()[min[0]]
    pc.points = o3d.utility.Vector3dVector(points)

    T = icp(pc, pc_gt)
    mesh.transform(T)
    pc_gt.transform(T)

    # post processing
    center = mesh.get_center()
    for g in [mesh, pc_gt, pc]:
        g.translate(-center)

    mesh = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
    o3d.visualization.draw_geometries([mesh, pc], mesh_show_wireframe=True, left=400)


if __name__ == "__main__":
    # TODO: add argparser
    main()