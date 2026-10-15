"""Geometry of the EuRoC VI-sensor stereo rig on the Crazyflie (-d euroc).

Shared by jinja_gen.py (camera poses in the SDF) and the dataset recorder
(calibration written for the VIO), so both use the same numbers.

Frames:
  body   Crazyflie base_link = IMU frame: x forward, y left, z up
  adis   EuRoC body frame = ADIS16448 IMU of the VI-sensor: x up, y right, z forward
         (looking along the stereo axis). The EuRoC T_BS / OpenVINS T_imu_cam of a
         camera is given in this frame.
  cam    optical frame: x right, y down, z forward (OpenCV / Kalibr)
  gz     Gazebo camera sensor frame: looks along x, y left, z up

The VI-sensor IMU is placed at the Crazyflie IMU, so the camera positions relative
to the IMU are the EuRoC ones. The rig is pitched down by tilt_deg ("front-down
looking" in the EuRoC paper, about 20 deg from the Vicon calibration of V1_01).
Each camera gets a random mounting error (rotation in the camera frame) that is in
the rendered images but not in the nominal calibration.

Other rigs with the same cameras (-d corax, noise_profiles/corax.yaml) give each camera
T_body_cam instead: camera -> body directly in the Crazyflie frame, tilt_deg is not used.
"""
import math
import random

import numpy as np

# adis -> body for a level rig: adis x (up) = body z, adis y (right) = -body y, adis z (forward) = body x
R_BODY_ADIS_LEVEL = np.array([[0.0, 0.0, 1.0],
                              [0.0, -1.0, 0.0],
                              [1.0, 0.0, 0.0]])
# gz camera frame -> optical frame: gz x (view) = cam z, gz y (left) = -cam x, gz z (up) = -cam y
R_CAM_GZ = np.array([[0.0, -1.0, 0.0],
                     [0.0, 0.0, -1.0],
                     [1.0, 0.0, 0.0]])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_to_matrix(roll, pitch, yaw):
    """SDF convention: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def matrix_to_rpy(R):
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def orthonormalize(R):
    u, _, vt = np.linalg.svd(R)
    return u @ vt


def r_body_adis(cfg):
    """adis -> body, including the downward tilt of the rig (positive pitch = nose down)."""
    return rot_y(math.radians(float(cfg.get('tilt_deg') or 0.0))) @ R_BODY_ADIS_LEVEL


def mount_errors(cfg, seed, cf_id, names):
    """Random roll/pitch/yaw [rad] per camera, uniform in +-mount_error_deg, drawn per drone."""
    limit = float(cfg.get('mount_error_deg') or 0.0)
    errors = {}
    for i, name in enumerate(names):
        rng = random.Random(int(seed) * 1000 + int(cf_id) * 10 + i + 500)
        errors[name] = [math.radians(rng.uniform(-limit, limit)) for _ in range(3)] if limit else [0.0] * 3
    return errors


def camera_names(cfg):
    return sorted((cfg.get('cameras') or {}).keys())


def body_cam(cam, R_ba):
    """Nominal camera -> body transform of one camera: T_body_cam as given (Crazyflie body frame,
    e.g. the Corax rig), or T_imu_cam in the EuRoC IMU frame rotated by R_ba (adis -> body, with tilt)."""
    T = np.eye(4)
    if cam.get('T_body_cam') is not None:
        T_body_cam = np.array(cam['T_body_cam'], float).reshape(4, 4)
        T[:3, :3] = orthonormalize(T_body_cam[:3, :3])
        T[:3, 3] = T_body_cam[:3, 3]
        return T
    T_adis_cam = np.array(cam['T_imu_cam'], float).reshape(4, 4)
    T[:3, :3] = R_ba @ orthonormalize(T_adis_cam[:3, :3])
    T[:3, 3] = R_ba @ T_adis_cam[:3, 3]
    return T


def rig(cfg, seed=0, cf_id=0):
    """Per camera: nominal and true camera -> body transforms and the Gazebo sensor pose.

    Returns {name: {'T_body_cam': 4x4 nominal, 'T_body_cam_true': 4x4 with the mounting
    error, 'mount_error_rpy': [r, p, y] rad, 'gz_pose': 'x y z roll pitch yaw'}}.
    """
    R_ba = r_body_adis(cfg)
    names = camera_names(cfg)
    errors = mount_errors(cfg, seed, cf_id, names)
    out = {}
    for name in names:
        T = body_cam(cfg['cameras'][name], R_ba)
        T_true = T.copy()
        T_true[:3, :3] = T[:3, :3] @ rpy_to_matrix(*errors[name])
        R_gz = T_true[:3, :3] @ R_CAM_GZ
        x, y, z = T_true[:3, 3]
        out[name] = {
            'T_body_cam': T,
            'T_body_cam_true': T_true,
            'mount_error_rpy': errors[name],
            'gz_pose': '%.5f %.5f %.5f %.6f %.6f %.6f' % (x, y, z, *matrix_to_rpy(R_gz)),
        }
    return out


def render_size(cfg):
    """Gazebo render size: stream size times render_scale."""
    w, h = cfg['stream_size']
    scale = float(cfg.get('render_scale') or 1.0)
    return int(round(w * scale)), int(round(h * scale))
