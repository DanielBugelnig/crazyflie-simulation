#!/usr/bin/env python3

import jinja2
import argparse
import os
import shutil
import fnmatch
import json
import math
import random
import sys
import numpy as np
import yaml

# euroc_rig.py (EuRoC stereo rig geometry) is shared with the dataset recorder
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import euroc_rig  # noqa: E402

# Nominal masses [kg]; a noise profile overrides them with measured values.
# body = None keeps the default of the vehicle model.
NOMINAL_MASS = {'body': None, 'aideck': 0.0044, 'flowdeck': 0.0016, 'multiranger': 0.0023, 'euroc': 0.0001}

# Flow deck model (FlowDeckModel::Config in the plugin) without a profile: a
# well textured floor, no ToF noise and no per-drone errors. The PMW3901 keeps
# 1 count of noise: a noise free counting sensor reads 0 in a still hover, the
# driver drops reads without motion, and the estimator starves (limit cycle).
FLOWDECK_DEFAULTS = {
    'tofRate': 40, 'tofMaxRange': 3.0, 'tofNoise': 'false', 'tofInvalidDistance': 8.19,
    'flowRate': 100, 'flowNoise': 1.0, 'flowScaleErrorMax': 0.0, 'flowYawMisalignmentMax': 0.0,
    'flowMinHeight': 0.08, 'squalMean': 100, 'squalStd': 0, 'squalThreshold': 15,
    'shutterMean': 500, 'shutterStd': 0, 'qualityTau': 0.3, 'seed': 0,
}

# Deck flags reported to the firmware (SIM_DECK_* in CrtpUtils.h)
DECK_FLOW2, DECK_MULTIRANGER, DECK_AI = 1 << 0, 1 << 1, 1 << 2

# Visibility flag of the drone's own visuals, hidden from the EuRoC cameras
OWN_VISUAL_FLAG = 1 << 1


def get_file_contents(filepath):
    with open(filepath, 'rb') as f:
        return f.read()

def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def load_noise_profile(name, env_dir):
    """Load a noise profile given as a path or as a name in <env_dir>/noise_profiles."""
    if not name:
        return {}
    candidates = [name, os.path.join(env_dir, 'noise_profiles', name),
                  os.path.join(env_dir, 'noise_profiles', name + '.yaml')]
    for path in candidates:
        if os.path.isfile(path):
            with open(path) as f:
                return yaml.safe_load(f) or {}
    raise FileNotFoundError("noise profile '{}' not found (tried {})".format(name, ', '.join(candidates)))

def per_axis(value):
    return [float(v) for v in value] if isinstance(value, (list, tuple)) else [float(value)] * 3

def imu_parameters(profile, cf_id):
    """IMU noise from the profile's imu section.

    mode sdf: per-sample stddev for the Gazebo <noise> elements. A white sequence
    sampled at fs with stddev N*sqrt(fs/2) has the one-sided density N, so the
    noise that passes the firmware's gyro/accel low-pass filters matches the
    datasheet. mode plugin: parameters of the plugin's BMI088 model.
    """
    cfg = profile.get('imu') or {}
    imu = {'mode': 'off', 'rate_hz': int(cfg.get('rate_hz', 1000))}
    if not cfg.get('enabled', False):
        return imu
    mode = cfg.get('mode', 'sdf')
    if mode not in ('sdf', 'plugin'):
        raise ValueError("imu.mode '{}' is not supported (sdf or plugin)".format(mode))
    imu['mode'] = mode
    imu['seed'] = int(profile.get('seed', 0)) + int(cf_id) + 1000
    number = lambda value: value if value is not None else 0  # noqa: E731
    for key in ('gyro', 'accel'):
        sensor = cfg.get(key) or {}
        density = per_axis(sensor.get('noise_density', 0.0))
        bias = per_axis(sensor.get('turn_on_bias_std', 0.0))
        imu[key] = {
            'stddev': ['%.4g' % (n * math.sqrt(imu['rate_hz'] / 2.0)) for n in density],
            'bias_stddev': ['%.4g' % b for b in bias],
            'plugin': {
                'noiseDensity': ' '.join('%.4g' % n for n in density),
                'bandwidth': number(sensor.get('bandwidth_hz')),
                'turnOnBiasStd': ' '.join('%.4g' % b for b in bias),
                'biasRandomWalk': number(sensor.get('bias_random_walk')),
                'scaleErrorMax': number(sensor.get('scale_error_max')),
                'misalignmentMax': '%.5f' % math.radians(number(sensor.get('misalignment_deg'))),
                'warmupAmplitude': number(sensor.get('warmup_amplitude')),
                'warmupTau': number(sensor.get('warmup_tau_s')),
            },
        }
    vib = cfg.get('vibration') or {}
    imu['vibration'] = {
        'enabled': 'true' if vib.get('enabled', False) else 'false',
        'refRotVelocity': number(vib.get('ref_rot_velocity')) or 2000,
        'accel1x': number(vib.get('accel_1x')),
        'accel2x': number(vib.get('accel_2x')),
        'gyro1x': number(vib.get('gyro_1x')),
        'gyro2x': number(vib.get('gyro_2x')),
        'zRatio': number(vib.get('z_ratio')),
        'motorSpread': number(vib.get('motor_spread')),
    }
    return imu

def flowdeck_parameters(profile, cf_id):
    """Plugin parameters of the flow deck from the profile's flowdeck section."""
    params = dict(FLOWDECK_DEFAULTS)
    params['seed'] = int(profile.get('seed', 0)) + int(cf_id)
    cfg = profile.get('flowdeck') or {}
    if not cfg.get('enabled', False):
        return params
    tof, flow, floor = cfg.get('tof') or {}, cfg.get('flow') or {}, cfg.get('floor') or {}
    mapping = [
        (tof, 'max_range_m', 'tofMaxRange', float),
        (tof, 'noise', 'tofNoise', lambda v: 'true' if v else 'false'),
        (tof, 'invalid_distance_m', 'tofInvalidDistance', float),
        (flow, 'noise_counts', 'flowNoise', float),
        (flow, 'scale_error_max', 'flowScaleErrorMax', float),
        (flow, 'yaw_misalignment_deg', 'flowYawMisalignmentMax', lambda v: '%.5f' % math.radians(v)),
        (flow, 'min_height_m', 'flowMinHeight', float),
        (floor, 'squal_mean', 'squalMean', float),
        (floor, 'squal_std', 'squalStd', float),
        (floor, 'squal_threshold', 'squalThreshold', float),
        (floor, 'shutter_mean', 'shutterMean', float),
        (floor, 'shutter_std', 'shutterStd', float),
        (floor, 'quality_tau_s', 'qualityTau', float),
    ]
    for section, key, name, convert in mapping:
        if section.get(key) is not None:
            params[name] = convert(section[key])
    return params

def camera_parameters(profile, cf_id):
    """AI-deck camera geometry from the profile's camera section (defaults: today's camera)."""
    params = {'fps': 60, 'hfov': 1.50, 'rpy': '0 0 0', 'width': 324, 'height': 244}
    cfg = profile.get('camera') or {}
    if not cfg.get('enabled', False):
        return params
    params['fps'] = cfg.get('capture_fps') or params['fps']
    params['hfov'] = cfg.get('horizontal_fov') or params['hfov']
    # render larger than the stream (render_scale), resampled down in crazysim_cpx.py
    stream_w, stream_h = cfg.get('stream_size') or (params['width'], params['height'])
    scale = float(cfg.get('render_scale') or 1.0)
    params['width'], params['height'] = int(round(stream_w * scale)), int(round(stream_h * scale))
    mount_error = cfg.get('mount_error_deg')
    if mount_error:
        rng = random.Random(int(profile.get('seed', 0)) * 1000 + int(cf_id))
        rpy = [math.radians(rng.uniform(-mount_error, mount_error)) for _ in range(3)]
        params['rpy'] = ' '.join('%.5f' % a for a in rpy)
        print('camera mounting error (roll, pitch, yaw): {} deg'.format(
            ', '.join('%+.2f' % math.degrees(a) for a in rpy)))
    return params

def euroc_parameters(profile, cf_id):
    """EuRoC stereo rig (-d euroc) from the profile's euroc section: sensor poses and render settings."""
    cfg = profile.get('euroc') or {}
    if not cfg.get('enabled', False):
        raise ValueError("-d euroc needs a profile with an enabled euroc section (e.g. -p euroc)")
    width, height = euroc_rig.render_size(cfg)
    cameras = []
    for name, cam in euroc_rig.rig(cfg, profile.get('seed', 0), cf_id).items():
        cameras.append({'name': name, 'pose': cam['gz_pose']})
        print('{} mounting error (roll, pitch, yaw): {} deg'.format(
            name, ', '.join('%+.2f' % math.degrees(a) for a in cam['mount_error_rpy'])))
    return {'cameras': cameras, 'fps': cfg.get('capture_fps') or 20, 'hfov': cfg['horizontal_fov'],
            'width': width, 'height': height, 'hide_own_drone': bool(cfg.get('hide_own_drone', True))}

def read_jinja_parameters_from_file(filepath):
    if not filepath:
        return {}
    if not os.path.exists(filepath):
        return {}
    with open(filepath) as f:
        data = json.load(f)
        return data
    return {}

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', help="file that the sdf file should be generated from")
    parser.add_argument('env_dir')
    parser.add_argument('--cf_id', default=0, help="Crazyflie unique identifier number")
    parser.add_argument('--cffirm_udp_port', default=19950, help="Crazyflie Firmware UDP port")
    parser.add_argument('--cflib_udp_port', default=19850, help="Crazyflie python library UDP port")
    parser.add_argument('--cf_name', default="cf", help="Serial device for FMU")
    parser.add_argument('--camera', type=str2bool, default=False, help="Add the AI-deck camera sensor")
    parser.add_argument('--cam_frame_port', default=5200, help="UDP port the camera frames are sent to (crazysim_cpx.py)")
    parser.add_argument('--flowdeck', type=str2bool, default=False, help="Add the flow deck v2 (VL53L1x + PMW3901); the drone then flies without external pose")
    parser.add_argument('--euroc', type=str2bool, default=False, help="Add the EuRoC VI-sensor stereo cameras (profile euroc section)")
    parser.add_argument('--multiranger', type=str2bool, default=False, help="Add the Multi-ranger deck (mass only for now)")
    parser.add_argument('--noise_profile', default='', help="YAML noise profile (path or name in noise_profiles/)")
    parser.add_argument('--output-file', help="sdf output file")
    parser.add_argument('--stdout', action='store_true', default=False, help="dump to stdout instead of file")
    # parser.add_argument('--override_parameters_json_path', default='', help="json file with variables to override jinja parameters")
    args = parser.parse_args()
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(args.env_dir))
    template = env.get_template(os.path.relpath(args.filename, args.env_dir))

    d = {'np': np, \
         'cf_id': args.cf_id, \
         'cffirm_udp_port': args.cffirm_udp_port, \
         'cflib_udp_port': args.cflib_udp_port, \
         'cf_name': args.cf_name, \
         'camera': args.camera, \
         'cam_frame_port': args.cam_frame_port}

    profile = load_noise_profile(args.noise_profile, args.env_dir)
    mass = dict(NOMINAL_MASS)
    mass.update({k: v for k, v in (profile.get('mass') or {}).items() if v is not None})
    d['mass'] = mass
    d['imu'] = imu_parameters(profile, args.cf_id)
    d['flowdeck'] = args.flowdeck
    d['multiranger'] = args.multiranger
    d['euroc'] = euroc_parameters(profile, args.cf_id) if args.euroc else None
    # Visuals of this drone carry OWN_VISUAL_FLAG; the EuRoC cameras mask it out, other
    # cameras (GUI, AI-deck) keep the default mask and still see the drone
    d['own_visual'] = '<visibility_flags>{}</visibility_flags>'.format(OWN_VISUAL_FLAG) \
        if d['euroc'] and d['euroc']['hide_own_drone'] else ''
    d['own_visual_mask'] = 0x3FFFFFFF & ~OWN_VISUAL_FLAG  # ogre2 reserves the top two bits
    d['deck_flags'] = ((DECK_AI if args.camera else 0) | (DECK_FLOW2 if args.flowdeck else 0) |
                       (DECK_MULTIRANGER if args.multiranger else 0))
    d['flowdeck_params'] = flowdeck_parameters(profile, args.cf_id)
    d['camera_params'] = camera_parameters(profile, args.cf_id)
    # With the flow deck the estimator relies on flow + ToF like the real drone
    d['send_ext_pose'] = not args.flowdeck

    # override_params_path = args.override_parameters_json_path
    # if not override_params_path:
    #     override_params_path = args.env_dir + "/resources/px4_gazebo_jinja_parameters.json"

    # parameters_from_json_file = read_jinja_parameters_from_file(override_params_path)
    # d.update(parameters_from_json_file)
    result = template.render(d)

    if args.stdout:
        print(result)

    else:
        if args.output_file:
            filename_out = args.output_file
        else:
            if not args.filename.endswith('.sdf.jinja'):
                raise Exception("ERROR: Output file can only be determined automatically for " + \
                                "input files with the .sdf.jinja extension")
            filename_out = args.filename.replace('.sdf.jinja', '.sdf')
            assert filename_out != args.filename, "Not allowed to overwrite template"

        # Overwrite protection mechanism: after generation, the file will be copied to a "last_generated" file.
        # In the next run, we can check whether the target file is still unmodified.
        filename_out_last_generated = filename_out + '.last_generated'

        if os.path.exists(filename_out) and os.path.exists(filename_out_last_generated):
            # Check whether the target file is still unmodified.
            if get_file_contents(filename_out).strip() != get_file_contents(filename_out_last_generated).strip():
                raise Exception("ERROR: generation would overwrite changes to `{}`. ".format(filename_out) + \
                                "Changes should only be made to the template file `{}`. ".format(args.filename) + \
                                "Remove `{}` ".format(os.path.basename(filename_out)) + \
                                "(after extracting your changes) to disable this overwrite protection.")

        with open(filename_out, 'w') as f_out:
            print(('{:s} -> {:s}'.format(args.filename, filename_out)))
            f_out.write(result)

        # Copy the contents to a "last_generated" file for overwrite protection check next time.
        shutil.copy(filename_out, filename_out_last_generated)