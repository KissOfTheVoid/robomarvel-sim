"""Движение робота: размеры, пределы скоростей и формулы перевода.

Здесь собрано всё, что нужно, чтобы из команды корпусу получить скорости
колёс и наоборот, а также чтобы посчитать, где робот окажется через
заданное время. Ни ROS, ни топиков в этом файле нет: на входе числа,
на выходе числа.
"""

import math

WHEEL_RADIUS = 0.021  # m
WHEEL_BASE = 0.128  # m
MAX_LINEAR = 0.3  # m/s
MAX_ANGULAR = 2.5  # rad/s
MAX_ACCEL_LINEAR = 0.1  # m/s^2
MAX_ACCEL_ANGULAR = 5.0  # rad/s^2
MAX_DECEL_LINEAR = 2.0  # m/s^2
MAX_DECEL_ANGULAR = 5.0  # rad/s^2
MAX_WHEEL_SPEED = 35.0  # rad/s

CONTROL_HZ = 30
ANGULAR_GAIN = 1.8
LINEAR_GAIN = 2.0

MIN_INPLACE_ANGULAR = 0.8
INPLACE_ANGULAR_THRESHOLD = 0.1
INPLACE_LINEAR_THRESHOLD = 0.05

CMD_TIMEOUT = 0.3  # s, столько ждём новую команду, потом останавливаемся


def shape_command(linear_x, angular_z):
    """Готовит команду от пульта: усиление, ограничение, доводка разворота на месте."""
    v = linear_x * LINEAR_GAIN
    w = angular_z * ANGULAR_GAIN

    v = math.copysign(min(abs(v), MAX_LINEAR), v)
    w = math.copysign(min(abs(w), MAX_ANGULAR), w)

    if abs(v) < INPLACE_LINEAR_THRESHOLD and abs(w) > INPLACE_ANGULAR_THRESHOLD:
        w = math.copysign(max(abs(w), MIN_INPLACE_ANGULAR), w)
    elif abs(w) < INPLACE_ANGULAR_THRESHOLD:
        w = 0.0

    return v, w


def ramp(current, target, accel_step, decel_step):
    """Один шаг изменения скорости в сторону нужной, не длиннее разрешённого."""
    step = accel_step if abs(target) > abs(current) else decel_step
    if target > current:
        return min(current + step, target)
    return max(current - step, target)


def ramp_body(current_v, current_w, target_v, target_w, hz=CONTROL_HZ):
    v = ramp(current_v, target_v, MAX_ACCEL_LINEAR / hz, MAX_DECEL_LINEAR / hz)
    w = ramp(current_w, target_w, MAX_ACCEL_ANGULAR / hz, MAX_DECEL_ANGULAR / hz)
    return v, w


def body_to_wheels(v, w):
    """Из скоростей корпуса в угловые скорости колёс (рад/с), не выходя за предел мотора."""
    v_left = (v - w * WHEEL_BASE / 2) / WHEEL_RADIUS
    v_right = (v + w * WHEEL_BASE / 2) / WHEEL_RADIUS
    peak = max(abs(v_left), abs(v_right), 1e-9)
    scale = min(1.0, MAX_WHEEL_SPEED / peak)
    return v_left * scale, v_right * scale


def wheels_to_body(w_left, w_right):
    """Прямая кинематика: из угловых скоростей колёс обратно в скорости корпуса."""
    v = (w_left + w_right) * WHEEL_RADIUS / 2.0
    w = (w_right - w_left) * WHEEL_RADIUS / WHEEL_BASE
    return v, w


def integrate_pose(x, y, yaw, v, w, dt):
    """Где окажется робот через dt секунд. Шаг считается по дуге, а не по прямой."""
    if abs(w) < 1e-6:
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
    else:
        r = v / w
        yaw_new = yaw + w * dt
        x += r * (math.sin(yaw_new) - math.sin(yaw))
        y -= r * (math.cos(yaw_new) - math.cos(yaw))
        yaw = yaw_new
    return x, y, math.atan2(math.sin(yaw), math.cos(yaw))
