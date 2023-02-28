import json
import yaml
import numpy as np
from copy import deepcopy
from tf.transformations import quaternion_matrix, euler_matrix, quaternion_from_matrix, inverse_matrix

# Converting the output from k4a-calibration to the yaml format for easy_handeye

path = "/home/thomas/catkin_ws/src/k4a-calibration/build/cn0{}.json"
cams = [0, 1, 2, 3]

with open("/home/thomas/.ros/easy_handeye/0_eye_on_base.yaml_", "r") as f:
    cam0_yaml = yaml.load(f, Loader=yaml.SafeLoader)
T = cam0_yaml["transformation"]
T_bot_c0 = np.eye(4)
T_bot_c0 = quaternion_matrix(np.array([T["qx"], T["qy"], T["qz"], T["qw"]]))
T_bot_c0[:3, 3] = np.array([T["x"], T["y"], T["z"]])

# T_c0_cn = np.eye(4)  # placeholder
T_tag_c0 = np.eye(4)
for cam_index in cams:
    with open(path.format(cam_index), 'r') as f:
        data = json.load(f) 
    print(data)
    data = data['value0']


    q_orig = np.array([data['rotation']['x'], data['rotation']['y'], data['rotation']['z'], data['rotation']['w']])
    T_tag_cn = quaternion_matrix(q_orig)
    T_tag_cn[0, 3] = data['translation']['m00']
    T_tag_cn[1, 3] = data['translation']['m10']
    T_tag_cn[2, 3] = data['translation']['m20']

    if cam_index == 0:
        T_c0_tag = inverse_matrix(T_tag_cn)
        # T_tag_c0[:3, :3] = np.linalg.inv(T_tag_c0[:3, :3])
        # q = quaternion_from_matrix(T_tag_cn)
        # p = T_tag_cn[:3, 3]
        q = quaternion_from_matrix(T_bot_c0)
        p = T_bot_c0[:3, 3]
    else: 
        # transform to c0
        # print(T_cn_tag)
        # print(T_tag_c0)
        # T_cn_c0 = T_cn_tag @ T_tag_c0
        T_c0_cn = T_c0_tag @ T_tag_cn
        q = quaternion_from_matrix(T_c0_cn)
        p = T_c0_cn[:3, 3]

    base_frame = "panda_link0" if cam_index == 0 else "0_rgb_camera_link"
    # base_frame = "panda_link0" 

    camdict = {
        "parameters": {
            "eye_on_hand": False,
            "freehand_robot_movement": False,
            "move_group": "panda_arm",
            "move_group_namespace": "",
            "namespace": f"/{cam_index}_eye_on_base/",
            "robot_base_frame": base_frame,
            # "robot_base_frame": "panda_link0",
            # "robot_base_frame": "0_rgb_camera_link",
            "robot_effector_frame": "panda_hand",
            "tracking_base_frame": f"{cam_index}_rgb_camera_link",
            "tracking_marker_frame": "aruco_tag",
        },
        "transformation": {
            "qw": float(q[3]),
            "qx": float(q[0]),
            "qy": float(q[1]),
            "qz": float(q[2]),
            "x": float(p[0]),
            "y": float(p[1]),
            "z": float(p[2])
        },
    }

    fname = f"/home/thomas/.ros/easy_handeye/{cam_index}_eye_on_base.yaml"
    with open(fname, "w") as f:
        yaml.dump(camdict, f)

    # T_c0_cnm1 = T_cn_tag
    