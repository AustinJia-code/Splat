import open3d as o3d
import numpy as np
import cv2
import os

STEPS_PER_ROTATION = 120

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

width = int(cx * 2)
height = int(cy * 2)

intrinsic = o3d.camera.PinholeCameraIntrinsic (width, height, fx, fy, cx, cy)

'''
Open3D's extrinsic is World-to-Camera, so we need to position the world 
s.t. the camera at origin looks at the points how they were imaged... :(
Points are at positive Z, so we need to transform them to be in front

'''
extrinsic = np.array ([
        [1,  0,  0, 0],
        [0,  1,  0, 0],
        [0,  0,  1, 0],
        [0,  0,  0, 1]
    ], dtype = np.float64)

folder = "./data/out-5-points/"
pcds = []
# Add all point clouds
for ply in os.listdir (folder):
    file_path = os.path.join (folder, ply)
    steps = int (ply.split ('.')[0])

    # All about z-axis
    angle = (steps / STEPS_PER_ROTATION) * 2 * np.pi
    pcd = o3d.io.read_point_cloud (file_path)
    rotation_matrix = np.array (
        [[np.cos (angle), 0, np.sin (angle), 0],
        [0,               1, 0,              0],
        [-np.sin (angle), 0, np.cos (angle), 0],
        [0,               0, 0,              1]])
    pcd.transform (rotation_matrix)
    pcds.append (pcd)

    print (f"Loaded {ply}")

merged_pcd = o3d.geometry.PointCloud ()
for pcd in pcds:
    merged_pcd += pcd
merged_pcd = merged_pcd.voxel_down_sample (voxel_size = 5.0)
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