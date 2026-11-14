"""Walking home pose saved from the control page, plus its gravity reference."""
from __future__ import annotations
import json
import math
import os
from pathlib import Path
import tempfile

from rl_core import (ACTION_DIM, HOME_DEG, _unrotate_imu_bias, gravity_attitude_deg)

STANCE_PATH = Path(__file__).resolve().parents[1] / '.cache' / 'stance.json'
STANCE_LIMIT_DEG = 60.0
CAPTURE_SECONDS = 2.0
MIN_CAPTURE_SAMPLES = 40
# Policy order: left leg, neck/head, right leg. Mouth is not part of the 14.
JOINTS = (
    (10, 'Left hip yaw'),
    (11, 'Left hip roll'),
    (12, 'Left hip pitch'),
    (13, 'Left knee'),
    (14, 'Left ankle'),
    (30, 'Neck pitch'),
    (31, 'Head pitch'),
    (32, 'Head yaw'),
    (33, 'Head roll'),
    (20, 'Right hip yaw'),
    (21, 'Right hip roll'),
    (22, 'Right hip pitch'),
    (23, 'Right knee'),
    (24, 'Right ankle'),
)
assert len(JOINTS) == ACTION_DIM

# Walk stance that stands on this duck. Reset restores this pair.
# Hips are Pollen STAND2; ankles were eased from ±26° to ±18° so the
# trunk stopped sitting behind the feet. Gravity reference is the still
# capture taken in that pose. Get-up keeps HOME_DEG (±22°) separately.
BASELINE_HOME_DEG = [
    0.0, -5.0, -26.24, -0.28, 18.0,
    20.0, 20.0, 0.0, 0.0,
    0.0, 5.0, 26.24, 0.28, -18.0,
]
BASELINE_IMU_BIAS_DEG = (1.641491864495735, 2.185191871622685, 0.0)


def factory_home():
    return [float(v) for v in HOME_DEG]


def baseline_home():
    return [float(v) for v in BASELINE_HOME_DEG]


def baseline_bias():
    return tuple(BASELINE_IMU_BIAS_DEG)


def _numbers(values, length, shape_message, limit, limit_message):
    if isinstance(values, (str, bytes)) or not hasattr(values, '__len__') or len(values) != length:
        raise ValueError(shape_message)
    result = []
    for value in values:
        if isinstance(value, bool):
            raise ValueError(limit_message)
        try:
            value = float(value)
        except (TypeError, ValueError):
            raise ValueError(limit_message)
        if not math.isfinite(value) or abs(value) > limit:
            raise ValueError(limit_message)
        result.append(value)
    return result


def parse_home(values):
    return _numbers(values, ACTION_DIM, 'standing pose needs 14 joint degrees',
                    STANCE_LIMIT_DEG, f'each joint must stay within ±{STANCE_LIMIT_DEG:.0f}°')


def parse_bias(values):
    return _numbers(values, 3, 'gravity reference needs roll, pitch and yaw',
                    45.0, 'gravity reference must stay within ±45°')


def load(path=STANCE_PATH):
    path = Path(path)
    try:
        record = json.loads(path.read_text(encoding='utf-8'))
    except FileNotFoundError:
        return None
    if not isinstance(record, dict) or record.get('version') != 1:
        raise ValueError('saved standing pose is from an older version')
    home = parse_home(record.get('home_deg'))
    bias = parse_bias(record['imu_bias_deg']) if 'imu_bias_deg' in record else None
    return dict(version=1, home_deg=home, imu_bias_deg=bias)


def save(path, home_deg, imu_bias_deg):
    record = dict(version=1, home_deg=parse_home(home_deg), imu_bias_deg=parse_bias(imu_bias_deg))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(record, ensure_ascii=False, indent=2) + '\n').encode('utf-8')
    fd, name = tempfile.mkstemp(prefix='.stance-', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as output:
            output.write(payload)
            output.flush()
            os.fsync(output.fileno())
        os.replace(name, path)
    finally:
        Path(name).unlink(missing_ok=True)
    return record


def delete(path):
    path = Path(path)
    try:
        path.unlink()
    except FileNotFoundError:
        return


def bias_from_gravity(samples):
    """Attitude of a still pose. Each sample is post-rotation gravity and gyro rad/s."""
    if len(samples) < MIN_CAPTURE_SAMPLES:
        raise ValueError('Not enough IMU readings. Keep the body still and try again.')
    units = []
    gyro_peak = 0.0
    gyro_sum = 0.0
    for grav, gyro in samples:
        raw = _unrotate_imu_bias(grav)
        norm = math.sqrt(sum(v * v for v in raw))
        if norm < 0.5 or not all(math.isfinite(v) for v in (*raw, *gyro)):
            raise ValueError('Gravity reading is not usable.')
        units.append([v / norm for v in raw])
        rate = math.degrees(math.sqrt(sum(v * v for v in gyro)))
        gyro_peak = max(gyro_peak, rate)
        gyro_sum += rate
    count = len(units)
    mean = [math.fsum(row[i] for row in units) / count for i in range(3)]
    mean_norm = math.sqrt(sum(v * v for v in mean))
    if mean_norm < 0.5:
        raise ValueError('Gravity reading is not usable.')
    mean = [v / mean_norm for v in mean]
    spread = max(
        math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(row, mean)))))
        for row in units)
    if math.degrees(spread) > 4.0 or gyro_peak > 15.0 or gyro_sum / count > 6.0:
        raise ValueError('Movement detected. Hold the body still and try again.')
    roll, pitch = gravity_attitude_deg(mean)
    if abs(roll) > 45.0 or abs(pitch) > 45.0:
        raise ValueError('The body is not upright enough to capture gravity.')
    return roll, pitch, 0.0
