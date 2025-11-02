'''
pip install open3d, opencv-python
'''

import open3d as o3d
import numpy as np
import cv2
import os

# Load calibration
yaml_file = "../src/config/stereo_calib.yml"
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

ply_file = "../data/out-5-points/0.ply"
if not os.path.exists (ply_file):
    raise FileNotFoundError (f"{ply_file} not found")

pcd = o3d.io.read_point_cloud (ply_file)
print (f"Loaded {len (pcd.points)} points from {ply_file}")

# View
vis = o3d.visualization.Visualizer ()
vis.create_window ("Stereo Camera View", width = width, height = height)
vis.add_geometry (pcd)

param = o3d.camera.PinholeCameraParameters ()
param.intrinsic = intrinsic
param.extrinsic = extrinsic

ctr = vis.get_view_control ()
# NEED allow_arbitrary or it sets you up weird
ctr.convert_from_pinhole_camera_parameters (param, allow_arbitrary = True)

vis.run ()
vis.destroy_window ()