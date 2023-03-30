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
from glpg_flowmeadow.transformations.methods import rot_mat

from definitions import *
from lib.helper.cloud_operations import draw_point_clouds, display_dist, construct_T
from lib.helper.lego_bricks import load_stl, get_base_bricks
from lib.recon.reconstruction import reconstruct_point_cloud
from lib.simulator.cloud_app import CloudApp
import open3d as o3d

from scripts.sim_recon_setup import prepare_mesh

from lib.retrieval.cloud_alignment import find_model, rate_alignment, show_results

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
    # o3d.visualization.draw_geometries([pc])

    dists = {}
    for B in Brick().all_bricks():
        mesh, pc_gt, _, _, _,_, brick_id = B
        # display_dist(pc.compute_point_cloud_distance(pc_gt), pc, mesh, mesh_only=True, coord_axes=True)

        T = icp(pc, pc_gt)
        mesh.transform(T)
        pc_gt.transform(T)

        # post processing
        center = mesh.get_center()
        for g in [mesh, pc_gt, pc]:
            g.translate(-center)
            # g.rotate(rot_mat((1., 0., 0.), -30), center)

        dists[brick_id] = pc.compute_point_cloud_distance(pc_gt)
        # display_dist(dist, pc, mesh, mesh_only=True, coord_axes=True)

        print(f"Mean distance to {brick_id}: {np.mean(dists[brick_id])}")
    return

    display_dist(pc.compute_point_cloud_distance(pc_gt), pc, mesh, mesh_only=True, coord_axes=True)

    T = icp(pc, pc_gt)
    mesh.transform(T)
    pc_gt.transform(T)

    # post processing
    center = mesh.get_center()
    for g in [mesh, pc_gt, pc]:
        g.translate(-center)
        # g.rotate(rot_mat((1., 0., 0.), -30), center)
    display_dist(pc.compute_point_cloud_distance(pc_gt), pc, mesh, mesh_only=True, coord_axes=True)
    
    return
    files, errors, transformations = find_model(
        pc_gt,
        debug_file=f"{brick_id}.stl",  # select a model file for alignment visualization
        max_best=10,  # maximum amount of files used from preselection
    )

    # rate the alignment and show results
    ret, idcs = rate_alignment(errors)
    if ret:
        print(f"Found {len(idcs)} possible models:")
        show_results(files[idcs], errors[idcs], transformations[idcs], pc_gt, mesh_only=True)
    else:
        print("Brick identification failed!")


    return
    # TODO: This is only for GIF creation
    frame = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    vis.add_geometry(pc)
    vis.add_geometry(frame)
    files = []

    img_num = 200
    full_rot = 360
    step = full_rot / img_num
    center = mesh.get_center()

    view_ctl = vis.get_view_control()
    cam = view_ctl.convert_to_pinhole_camera_parameters()
    R, T = cam.extrinsic[:3, :3], cam.extrinsic
    R = rot_mat((1, 0, 0), -45)
    T_cam = construct_T(R, np.zeros(3))
    T = T @ T_cam
    T[:3, 3] *= 2
    cam.extrinsic = T   # where T is your matrix

    for i in range(img_num):
        file = f"tmp_{i}.png"
        files.append(file)

        vis.add_geometry(pc)
        vis.add_geometry(frame)
        pc.rotate(rot_mat((0., 0., 1.), -step), center)
        frame.rotate(rot_mat((0., 0., 1.), -step), center)

        view_ctl.convert_from_pinhole_camera_parameters(cam)

        vis.capture_screen_image(file, do_render=True)

    # gif
    from PIL import Image
    imgs = [Image.open(file) for file in files]

    x_range, y_range = [np.inf, -np.inf], [np.inf, -np.inf]
    for img in imgs:
        arr = np.asarray(img)
        arr = np.mean(arr, axis=-1) - 255
        y_idcs, x_idcs = np.nonzero(arr)
        x_range = [min(np.min(x_idcs), x_range[0]), max(np.max(x_idcs), x_range[1])]
        y_range = [min(np.min(y_idcs), y_range[0]), max(np.max(y_idcs), y_range[1])]

    print(x_range, y_range)
    w, h = imgs[0].size
    v_off, h_off = 600, 300
    imgs = [img.crop([x_range[0], y_range[0], x_range[1], y_range[1]]) for img in imgs]

    imgs[0].save(f'real_{brick_id}.gif', save_all=True, append_images=imgs[1:], optimize=False, duration=40, loop=0)
    for file in files:
        os.remove(file)


if __name__ == "__main__":
    # TODO: add argparser

    # recon settings

    main()