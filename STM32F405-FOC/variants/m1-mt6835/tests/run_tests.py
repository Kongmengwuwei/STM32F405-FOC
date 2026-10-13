"""Build/run host regressions without accessing hardware or serial ports."""
import argparse
import itertools
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT.parents[1]
OUTER = ['App/Control/control.c', 'App/Control/anticogging.c']
FOC = OUTER + ['App/FOC/foc.c', 'App/FOC/motor_ident.c', 'App/FOC/sensor_cal.c']
INCLUDES = ['App/Control', 'App/FOC', 'App/Config', 'App/Protocols/JustFloat',
            'App/Hardware/bsp', 'App/Hardware/encoder', 'App/Hardware/mt6835']
LONGQIU = ['FOC_PORT_M1', 'FOC_ENCODER_TLE5012B', 'FOC_MOTOR_L4225_6S420_A',
           'FOC_LOAD_FREE_SHAFT', 'FOC_INSTALLATION_ID=4']
M0 = ['FOC_PORT_M0', 'FOC_ENCODER_TLE5012B', 'FOC_MOTOR_ZH3620_1', 'FOC_INSTALLATION_ID=1']
REFERENCE = ['FOC_PORT_M1', 'FOC_ENCODER_MT6835', 'FOC_MOTOR_REFERENCE_24V',
             'FOC_INSTALLATION_ID=1', 'FOC_PROTECTION_TRIP=1']


def check_defaults():
    """Presets explicitly pin cached hardware; reject entry-point drift."""
    defaults = dict(re.findall(r'set\((FOC_\w+) "([^"]+)" CACHE',
                              (PROJECT / 'cmake/foc-defaults.cmake').read_text()))
    for path in [PROJECT / 'CMakePresets.json', ROOT / 'CMakePresets.json']:
        presets = json.loads(path.read_text())['configurePresets']
        actual = next(p for p in presets if p['name'] == 'default')['cacheVariables']
        assert all(actual[k] == v for k, v in defaults.items()), path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default='gcc', help='host GCC executable or absolute path')
    args = parser.parse_args()
    cc = shutil.which(args.cc)
    if not cc:
        parser.error('Host GCC not found; use --cc <path>. ARM GCC cannot run host tests.')
    check_defaults()
    output = PROJECT / 'build/host-tests'
    output.mkdir(parents=True, exist_ok=True)
    cases = []

    def add(name, definitions, sources, includes=()):
        cases.append((name, definitions, sources, includes))

    for name in ['sensor_cal', 'foc_adaptive', 'longqiu', 'position_precision']:
        add(name, LONGQIU, [f'tests/test_{name}.c'] + FOC)
    add('longqiu_trip', LONGQIU + ['FOC_PROTECTION_TRIP=1'], ['tests/test_longqiu.c'] + FOC)
    add('motor_ident', LONGQIU, ['tests/test_motor_ident.c', 'App/FOC/motor_ident.c'])
    add('motor_record', LONGQIU, ['tests/test_motor_record.c'])
    add('app_usb', REFERENCE, ['tests/test_app_usb.c', 'App/Control/app.c'] + FOC)
    for load in ['PLASTIC_ARM', 'FREE_SHAFT']:
        add(f'm0_{load.lower()}', M0 + [f'FOC_LOAD_{load}'],
            ['tests/test_m0_unified.c', 'App/Control/app.c'] + FOC)
    for name in ['anticogging', 'longqiu_dynamic', 'control_observer', 'control_tune_scale']:
        add(name, LONGQIU, [f'tests/test_{name}.c', 'tests/foc_stub.c'] + OUTER)
    add('control_pid', REFERENCE, ['tests/test_control_pid.c', 'tests/foc_stub.c'] + OUTER)
    add('foc_recalibration', REFERENCE, ['tests/test_foc_recalibration.c'] + FOC)
    add('pwm_zero_offset', M0, ['tests/test_pwm_zero_offset.c'] + FOC)
    for port, encoder in itertools.product(['M0', 'M1'], ['TLE5012B', 'MT6835']):
        defs = [f'FOC_PORT_{port}', f'FOC_ENCODER_{encoder}', 'FOC_MOTOR_REFERENCE_24V',
                'FOC_INSTALLATION_ID=1']
        add(f'window_{port}_{encoder}', defs, ['tests/test_current_window.c'] + FOC)
        for motor in ['ZH3620_1', 'REFERENCE_24V', 'L4225_6S420_A']:
            add(f'profile_{port}_{encoder}_{motor}',
                [f'FOC_PORT_{port}', f'FOC_ENCODER_{encoder}', f'FOC_MOTOR_{motor}',
                 'FOC_INSTALLATION_ID=4'], ['tests/test_profile_matrix.c'])
    for dual, cycles in itertools.product([0, 1], [28, 56]):
        add(f'window_m0_dual{dual}_{cycles}',
            M0 + [f'FOC_M0_DUAL_ADC={dual}', f'FOC_M0_CURRENT_SAMPLE_CYCLES={cycles}'],
            ['tests/test_current_window.c'] + FOC)
    add('dual_leg', ['FOC_DUAL', 'FOC_DUAL_LEG', 'FOC_PORT_M0', 'FOC_ENCODER_TLE5012B',
                    'FOC_MOTOR_L4225_6S420_A', 'FOC_LOAD_FREE_SHAFT', 'FOC_INSTALLATION_ID=5',
                    'FOC_PROTECTION_TRIP=1'],
        ['tests/test_dual_leg.c', 'dual/app.c', 'dual/leg_guard.c'] +
        [f'dual/{unit}{i}.c' for unit in ['foc', 'control', 'ident', 'sensor', 'cog'] for i in [0, 1]],
        ['dual', 'tests/dual_stubs'])
    add('tle5012b_response', LONGQIU,
        ['tests/test_tle5012b_response.c', 'App/Hardware/tle5012b/tle5012b_protocol.c'],
        ['App/Hardware/tle5012b'])
    add('justfloat', LONGQIU, ['tests/test_justfloat.c'])
    add('mt6835_crc', LONGQIU, ['tests/test_mt6835_crc.c', 'App/Hardware/mt6835/mt6835.c'])
    add('usb_queue', LONGQIU, ['tests/test_usb_queue.c'], ['tests/usb_stubs'])
    for name, definitions, sources, extra_includes in cases:
        print(f'[{name}]', flush=True)
        exe = output / (name + ('.exe' if sys.platform == 'win32' else ''))
        includes = [part for path in [*extra_includes, *INCLUDES] for part in ['-I', path]]
        command = [cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-O2', *includes,
                   *(f'-D{d}' for d in definitions), *sources, '-lm', '-o', str(exe)]
        subprocess.run(command, cwd=ROOT, check=True, timeout=60)
        subprocess.run([str(exe)], cwd=ROOT, check=True, timeout=60)
    scripts = sorted(ROOT.glob('tests/test_*.py')) + [ROOT / 'tools/bench/selftest.py']
    for script in scripts:
        print(f'[{script.name}]', flush=True)
        subprocess.run([sys.executable, str(script)], cwd=ROOT, check=True, timeout=60)
    print(f'PASS: defaults, {len(cases)} C configurations, {len(scripts)} Python suites', flush=True)


if __name__ == '__main__':
    main()
