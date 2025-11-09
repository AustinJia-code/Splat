'''
Build point cloud assuming subject is rotating
'''

import open3d as o3d
import numpy as np
import cv2
import os

########################################
STEPS_PER_ROTATION = 200
Z_DIST = -865
R_TOL = 80
extrinsic = np.array ([
        [1,  0,  0, 0],
        [0,  1,  0, 0],
        [0,  0,  1, -Z_DIST],
        [0,  0,  0, 1]
    ], dtype = np.float64)
folder = "./data/out/pumpkin/5-points/"
out_path = ""   # out path for combined point cloud... not implemented
########################################

# Load calibration
yaml_file = "./src/config/stereo_calib.yml"
fs = cv2.FileStorage (yaml_file, cv2.FILE_STORAGE_READ)

K1 = fs.getNode ("K1").mat ()
D1 = fs.getNode ("D1").mat ()
R1 = fs.getNode ("R1").mat ()
P1 = fs.getNode ("P1").mat ()
fs.release ()

fx = P1[0, 0]
fy = P1[1, 1]
cx = P1[0, 2]
cy = P1[1, 2]

width = int (cx * 2)
height = int (cy * 2)

intrinsic = o3d.camera.PinholeCameraIntrinsic (width, height, fx, fy, cx, cy)

'''
Open3D's extrinsic is World-to-Camera, so we need to position the world 
s.t. the camera at origin looks at the points how they were imaged... :(
Points are at positive Z, so we need to transform them to be in front

'''
pcds = []
# Add all point clouds
# Add all point clouds
for ply in os.listdir (folder):
    file_path = os.path.join (folder, ply)
    steps = int (ply.split ('.')[0])
    angle = (steps / STEPS_PER_ROTATION) * 2 * np.pi

    # Read the point cloud
    pcd = o3d.io.read_point_cloud (file_path)

    # Move object so that its rotation axis is at origin
    T_to_origin = np.eye (4)
    T_to_origin[2, 3] = Z_DIST

    # Step 2: apply rotation of the *subject* (opposite direction)
    R_obj = np.array ([
        [ np.cos (-angle), 0, np.sin (-angle), 0],
        [ 0,              1, 0,              0],
        [-np.sin (-angle), 0, np.cos (-angle), 0],
        [ 0,              0, 0,              1]])

    # Step 3: move back to camera coordinates
    T_back = np.eye(4)
    T_back[2, 3] = -Z_DIST

    # Combine transformations: T_back * R_obj * T_to_origin
    transform = T_back @ R_obj @ T_to_origin
    pcd.transform (transform)
    points = np.asarray (pcd.points)
    colors = np.asarray (pcd.colors)

    # Distance Mask
    world_origin = [0, 100, -Z_DIST]
    distances = np.linalg.norm (points - world_origin, axis = 1)
    distance_mask = distances <= R_TOL

    # Color mask - filter out black points
    color_brightness = np.sum (colors, axis = 1)  # Sum of R+G+B
    color_mask = color_brightness > 0.0  # Threshold to exclude near-black
    # mask = distance_mask & color_mask
    mask = color_mask

    pcd.points = o3d.utility.Vector3dVector (points[mask])
    pcd.colors = o3d.utility.Vector3dVector (colors[mask])

    pcds.append (pcd)

    print(f"Loaded {ply} (angle = {np.degrees (angle):.1f}°)")

merged_pcd = o3d.geometry.PointCloud ()
for pcd in pcds:
    merged_pcd += pcd
# merged_pcd = merged_pcd.voxel_down_sample (voxel_size = 5.0)
print (len (merged_pcd.points))

# View
vis = o3d.visualization.Visualizer ()
vis.create_window ("Stereo Camera View", width = width, height = height)
vis.add_geometry (merged_pcd)

param = o3d.camera.PinholeCameraParameters ()
param.intrinsic = intrinsic
param.extrinsic = extrinsic

ctr = vis.get_view_control ()
# NEED allow_arbitrary or it sets you up weird
ctr.convert_from_pinhole_camera_parameters (param, allow_arbitrary = True)

vis.run ()
vis.destroy_window ()