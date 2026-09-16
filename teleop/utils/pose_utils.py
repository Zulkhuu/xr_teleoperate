"""Pure rigid-pose operations for controller anchoring."""

import numpy as np

def fast_pose_inv(pose):
    pose_inv = np.eye(4)
    pose_inv[:3, :3] = pose[:3, :3].T
    pose_inv[:3, 3] = -pose[:3, :3].T @ pose[:3, 3]
    return pose_inv

def align_wrist_pose_to_start(current_pose, xr_start_pose, robot_start_pose):
    return robot_start_pose @ fast_pose_inv(xr_start_pose) @ current_pose
