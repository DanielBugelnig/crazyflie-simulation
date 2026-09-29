#!/usr/bin/env python3
"""Set the mass dependent firmware parameters for a deck configuration.

The controllers only know the bare Crazyflie. With decks mounted they need:
  posCtlPid.thrustBase  PID: hover thrust feed forward (PWM)
  ctrlMel.mass          Mellinger: vehicle mass [kg]
  ctrlAtt.mass          Brescianini: vehicle mass [kg] (CrazySim firmware fork)

The masses come from the same noise profile the simulator uses, so the
simulated and the real drone get identical values. The hover PWM follows
from the firmware thrust curve T(p) = A p^2 + B p per motor (p = PWM/65535,
powerDist.pwmToThrustA/B); override it with --thrust-base once you have a
hover log of the real drone.

Command line (connects, sets the parameters, disconnects):
  python3 payload_params.py --decks aideck,flowdeck --dry-run
  python3 payload_params.py --uri udp://127.0.0.1:19850 --decks aideck,flowdeck
  python3 payload_params.py --uri radio://0/80/2M/E7E7E7E7E7 --decks aideck,flowdeck -p my_cf.yaml

Note: cflib's close_link() leaves the commander in low-level mode, so the next
connection has to call cf.commander.send_notify_setpoint_stop() before using
the high-level commander. Inside your own flight script use the module instead:
  from payload_params import payload_parameters, apply_parameters
  apply_parameters(scf.cf, payload_parameters(['aideck', 'flowdeck']))
"""
import argparse
import math
import os
import threading

import yaml

GRAVITY = 9.81
PWM_TO_THRUST_A = 0.091492681   # powerDist.pwmToThrustA (firmware default)
PWM_TO_THRUST_B = 0.067673604   # powerDist.pwmToThrustB (firmware default)
NOMINAL_MASS = {'body': 0.027, 'aideck': 0.0044, 'flowdeck': 0.0016, 'multiranger': 0.0023}
DECKS = ('aideck', 'flowdeck', 'multiranger')
PROFILE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'noise_profiles')


def load_masses(profile=None):
    """Masses [kg] from a noise profile (path or name in noise_profiles/)."""
    masses = dict(NOMINAL_MASS)
    if profile:
        for path in (profile, os.path.join(PROFILE_DIR, profile), os.path.join(PROFILE_DIR, profile + '.yaml')):
            if os.path.isfile(path):
                with open(path) as f:
                    cfg = yaml.safe_load(f) or {}
                masses.update({k: v for k, v in (cfg.get('mass') or {}).items() if v is not None})
                break
        else:
            raise FileNotFoundError("noise profile '{}' not found".format(profile))
    return masses


def total_mass(decks, masses):
    unknown = set(decks) - set(DECKS)
    if unknown:
        raise ValueError('unknown deck(s) {} (supported: {})'.format(', '.join(sorted(unknown)), ', '.join(DECKS)))
    return masses['body'] + sum(masses[d] for d in decks)


def hover_pwm(mass, a=PWM_TO_THRUST_A, b=PWM_TO_THRUST_B):
    """Motor PWM that carries mass [kg] with the firmware thrust curve."""
    thrust = mass * GRAVITY / 4.0
    p = (-b + math.sqrt(b * b + 4.0 * a * thrust)) / (2.0 * a)
    return int(round(p * 65535))


def payload_parameters(decks, profile=None, thrust_base=None):
    """Parameter name -> value for a deck configuration."""
    mass = total_mass(decks, load_masses(profile))
    return {
        'posCtlPid.thrustBase': int(thrust_base) if thrust_base else hover_pwm(mass),
        'ctrlMel.mass': round(mass, 5),
        'ctrlAtt.mass': round(mass, 5),
    }


def apply_parameters(cf, params, timeout=2.0):
    """Set params on a connected Crazyflie; skips names the firmware lacks."""
    for name, value in params.items():
        if cf.param.toc.get_element_by_complete_name(name) is None:
            print('  {:22s} not in this firmware, skipped'.format(name))
            continue
        group, par = name.split('.')
        updated = threading.Event()
        callback = lambda _name, _value: updated.set()  # noqa: E731
        cf.param.add_update_callback(group=group, name=par, cb=callback)
        cf.param.set_value(name, value)
        confirmed = updated.wait(timeout)
        cf.param.remove_update_callback(group=group, name=par, cb=callback)
        print('  {:22s} = {:<10} {}'.format(name, value, 'ok' if confirmed else 'NOT confirmed'))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--uri', default='udp://127.0.0.1:19850', help='Crazyflie URI (default: first sim drone)')
    parser.add_argument('-d', '--decks', default='', help='comma separated: ' + ', '.join(DECKS))
    parser.add_argument('-p', '--profile', default=None, help='noise profile with measured masses')
    parser.add_argument('--thrust-base', type=int, default=None, help='measured hover PWM instead of the model value')
    parser.add_argument('--dry-run', action='store_true', help='only print the values')
    args = parser.parse_args()

    decks = [d for d in args.decks.split(',') if d]
    params = payload_parameters(decks, args.profile, args.thrust_base)
    print('decks: {}  mass: {:.4f} kg'.format(', '.join(decks) or 'none', params['ctrlMel.mass']))
    if args.dry_run:
        for name, value in params.items():
            print('  {:22s} = {}'.format(name, value))
        return

    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    cflib.crtp.init_drivers()
    with SyncCrazyflie(args.uri, cf=Crazyflie()) as scf:
        apply_parameters(scf.cf, params)


if __name__ == '__main__':
    main()
