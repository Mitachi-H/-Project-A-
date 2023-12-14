import argparse
import os
import open3d as o3d # open3d == 0.16.0
import numpy as np
from scipy.spatial.transform import Rotation as R # scipy >= 1.4.0

def get_args():
    parser = argparse.ArgumentParser(description='Demo of open3d camera placement and rotation.')
    parser.add_argument('--filepath', type=str, help='The path to the point cloud file.')
    parser.add_argument('--width', type=int, default=1980, help='Width of the window.')
    parser.add_argument('--height', type=int, default=1080, help='Height of the window.')
    parser.add_argument('--point-size', type=int, default=1, help='The visulized size of each point.')
    parser.add_argument('--demo-mode', type=int, help='0: Place the camera at specific coordinates, 1: Rotate a specific angle around an object, 2: Rotate the camera at a specific angle.')
    parser.add_argument('--axis', type=str, help='Move & rotate axis.')
    parser.add_argument('--save-images', action='store_true', help='Save the rendered images or not.')

    return parser.parse_args()

def main(filepath, width, height, point_size, demo_mode, axis, save_images):
    # Read point cloud data from file.
    adjust_position = [0, -1, 1.2]
    pcd = read_point_cloud_pts(filepath, adjust_position)
    # Initialize the visualizer.
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=width, height=height)
    # Set the visulized size of each point.
    vis.get_render_option().point_size = point_size
    
    # Add point cloud data to the scene.
    vis.add_geometry(pcd)
    # Get camera extrinsic
    ctr = vis.get_view_control()

    # set camera
    ctr.set_lookat([0, 0, 0]) 
    # ctr.camera_local_rotate(0, 900)
    # ctr.camera_local_translate(0, 0, -6)
    ctr.set_zoom(1) 

    # Run the visualizer.
    vis.run()

    # Destroy the visualization window
    vis.destroy_window()

def read_point_cloud_pts(filepath, adjust_position, data_format='xyzirgb'):
    """Read point cloud data in pts format.

    Args:
        filepath (str): The point cloud data file (*.pts).
        data_format (str, optional): The data format of the pts file ('xyzirgb' or 'xyzrgb').
                                     Defaults to 'xyzirgb'.

    Returns:
        o3d.open3d.geometry.PointCloud: The point cloud data.
    """

    assert data_format in ['xyzirgb', 'xyzrgb']

    # Reads lines from the pts file.
    # The first line gives the number of points to follow.
    # Each subsequent line has 7 values, the first three are the (x,y,z) coordinates of the point,
    # the fourth is an "intensity" value (only avaiable in 'xyzirgb' format),
    # and the last three are the (r,g,b) colour values (range from 0 to 255).
    try:
        f = open(filepath)
        contents = f.readlines()
        f.close()
    except Exception as e:
        print(e)
        return None, None

    if len(contents) == 0:
        return None, None

    # Placeholders for points and colors.
    points = np.zeros((int(contents[0]), 3))
    colors = np.zeros((int(contents[0]), 3))

    for i in range(1, len(contents)):
        info = contents[i].strip().split(' ')

        for j in range(3):
            points[i-1, j] = float(info[j]) + adjust_position[j]
            skip_place = 1 if data_format == 'xyzirgb' else 0 # disregard the intensity value
            colors[i-1, j] = int(info[j+3+skip_place]) # colors

    pcd = o3d.open3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors / 255.)

    return pcd

filepath = 'pump1.pts'

width = 3000
height = 3000
point_size = 5
demo_mode = 1
axis = 'y'
save_images = True
#
main(filepath, width, height, point_size, demo_mode, axis, save_images)