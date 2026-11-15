"""Full compound support along the left fingertip axis, matching adapter.py."""
import numpy as np
from scipy.spatial.transform import Rotation

def distal_clearance(scene, body_position, body_quaternion, left_root, left_rotation):
    body_R = Rotation.from_quat(body_quaternion).as_matrix()
    axis = np.asarray(left_rotation)[:, 2]
    direction = body_R.T @ axis
    minima = []
    def box(center, rotation, size):
        minima.append(direction @ center - np.abs(direction @ rotation) @ (np.asarray(size)/2))
    box(np.zeros(3), scene.R, (.032, .032, .002))
    for name, center, size in scene.PARTS:
        rotation = scene.R
        if name.startswith('tube_'):
            rotation = rotation @ Rotation.from_euler('z', int(name.split('_')[1])*2*np.pi/12).as_matrix()
        box(scene.R @ (np.asarray(center)-scene.ROOT_LOCAL), rotation, size)
    local_axis = scene.R.T @ direction
    cylinder_extent = .0045*np.linalg.norm(local_axis[:2]) + .00175*abs(local_axis[2])
    for center in scene.STUDS:
        minima.append(direction @ (scene.R @ (np.asarray(center)-scene.ROOT_LOCAL)) - cylinder_extent)
    return float(axis @ (np.asarray(body_position)-left_root) + min(minima) - .146843)
