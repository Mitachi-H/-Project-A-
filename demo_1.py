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
    pcd = read_point_cloud_pts(filepath)

    # Initialize the visualizer.
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=width, height=height)

    # Set the visulized size of each point.
    vis.get_render_option().point_size = point_size

    # Add point cloud data to the scene.
    vis.add_geometry(pcd)

    # Get camera extrinsic
    ctr = vis.get_view_control()
    param = ctr.convert_to_pinhole_camera_parameters()
    ori_extrinsic = param.extrinsic.copy()

    # Please refer to the following description of open3d camera extrinsic and intrinsic parameters (it's in Japanese)
    # https://zenn.dev/fastriver/articles/open3d-camera-pinhole
    if demo_mode == 0:
        # Place the camera at specific coordinates
        if save_images:
            save_dir = 'place_{}'.format(axis)
            if save_dir and not os.path.exists(save_dir):
                os.makedirs(save_dir)
        axis_idxs = {'x': 0, 'y': 1, 'z': 2}
        for i in range(10):
            # Modify the extrinsic parameters of the camera to place it in a specific position
            extrinsic = param.extrinsic.copy()
            extrinsic[axis_idxs[axis], 3] += 1
            param.extrinsic = extrinsic
            ctr.convert_from_pinhole_camera_parameters(param)
            if save_images:
                vis.poll_events()
                vis.update_renderer()
                vis.capture_screen_image(os.path.join(save_dir, 'place_{}.png'.format(i)))
            else:
                draw_camera(vis, width, height)
    elif demo_mode == 1:
        # Rotate a specific angle around an object
        if save_images:
            save_dir = "demo"
            if save_dir and not os.path.exists(save_dir):
                os.makedirs(save_dir)
        # Calculate the rotation matrix
        # ------------------------------
        rot_1 = np.eye(4)
        rot_2 = np.eye(4)
        rot_1[:3, :3] = R.from_euler("x", 270, degrees=True).as_matrix()
        rot_2[:3, :3] = R.from_euler("z", -90, degrees=True).as_matrix()
        rot_2 = np.dot(rot_1, rot_2)
        for i, rot in enumerate([rot_1, rot_2]):
            param.extrinsic = np.dot(ori_extrinsic, rot)
            ctr.convert_from_pinhole_camera_parameters(param)
            if save_images:
                vis.poll_events()
                vis.update_renderer()
                vis.capture_screen_image(os.path.join(save_dir, 'rotate_a_{}.png'.format(i)))
            else:
                draw_camera(vis, width, height)
        # ------------------------------


    elif demo_mode == 2:
        # Rotate the camera at a specific angle
        if save_images:
            save_dir = 'rotate_s_{}'.format(axis)
            if save_dir and not os.path.exists(save_dir):
                os.makedirs(save_dir)
        for i in range(0, 360, 30):
            # Calculate the rotation matrix
            rot = np.eye(4)
            rot[:3, :3] = R.from_euler(axis, i, degrees=True).as_matrix()
            param.extrinsic = np.dot(rot, ori_extrinsic)
            ctr.convert_from_pinhole_camera_parameters(param)
            if save_images:
                vis.poll_events()
                vis.update_renderer()
                vis.capture_screen_image(os.path.join(save_dir, 'rotate_s_{}.png'.format(i)))
            else:
                draw_camera(vis, width, height)
    
    # Run the visualizer.
    vis.run()

    # Destroy the visualization window
    vis.destroy_window()

def read_point_cloud_pts(filepath, data_format='xyzirgb'):
    adjust_position = [0, -1, 1.2]
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

def draw_camera(visualizer, width, height, **kwargs):
    """_summary_

    Args:
        visualizer (o3d.visualization.Visualizer): The point cloud visualizer.
        width (int): Width of the window. 
        height (int): Height of window. 
        scale (int): camera model scale. Defaults to 1.
        color (list): color of the image plane and pyramid lines. Defaults to None.
    """    
    assert visualizer and width and height
    scale = kwargs.get('scale', 1)
    color = kwargs.get('color', None)

    ctr = visualizer.get_view_control()
    param = ctr.convert_to_pinhole_camera_parameters()
    intrinsic = param.intrinsic
    extrinsic = param.extrinsic

    # convert extrinsics matrix to rotation and translation matrix
    extrinsic = np.linalg.inv(extrinsic)
    R = extrinsic[0:3, 0:3]
    t = extrinsic[0:3, 3]

    geometries = draw_camera_geometries(intrinsic.intrinsic_matrix, R, t, width, height, scale, color)
    for g in geometries:
        visualizer.add_geometry(g)

#
# Auxiliary funcions
#
def draw_camera_geometries(K, R, t, width, height, scale=1, color=None):
    """ Create axis, plane and pyramid geometries in Open3D format
    :   param K     : calibration matrix (camera intrinsics)
    :   param R     : rotation matrix
    :   param t     : translation
    :   param width : image width
    :   param height: image height
    :   param scale : camera model scale
    :   param color : color of the image plane and pyramid lines
    :   return      : camera model geometries (axis, plane and pyramid)
    """

    # default color
    if color is None:
        color = [0.8, 0.2, 0.8]

    # camera model scale
    s = 1 / scale

    # intrinsics
    Ks = np.array([[K[0, 0] * s,            0, K[0,2]],
                   [          0,  K[1, 1] * s, K[1,2]],
                   [          0,            0, K[2,2]]])
    Kinv = np.linalg.inv(Ks)

    # 4x4 transformation
    T = np.column_stack((R, t))
    T = np.vstack((T, (0, 0, 0, 1)))

    # axis
    axis = create_coordinate_frame(T, scale=scale*0.5)

    # points in pixel
    points_pixel = [
        [0, 0, 0],
        [0, 0, 1],
        [width, 0, 1],
        [0, height, 1],
        [width, height, 1],
    ]

    # pixel to camera coordinate system
    points = [scale * Kinv @ p for p in points_pixel]

    # image plane
    width = abs(points[1][0]) + abs(points[3][0])
    height = abs(points[1][1]) + abs(points[3][1])
    plane = o3d.geometry.TriangleMesh.create_box(width, height, depth=1e-6)
    plane.paint_uniform_color(color)
    plane.transform(T)
    plane.translate(R @ [points[1][0], points[1][1], scale])

    # pyramid
    points_in_world = [(R @ p + t) for p in points]
    lines = [
        [0, 1],
        [0, 2],
        [0, 3],
        [0, 4],
    ]
    colors = [color for i in range(len(lines))]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points_in_world),
        lines=o3d.utility.Vector2iVector(lines))
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # return as list in Open3D format
    return [axis, plane, line_set]


def create_coordinate_frame(T, scale=0.25):
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=scale)
    frame.transform(T)
    return frame


filepath = 'pump1.pts'
width = 3000
height = 3000
point_size = 5
demo_mode = 1
axis = 'z'
save_images = True

main(filepath, width, height, point_size, demo_mode, axis, save_images)