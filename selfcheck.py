#!/usr/bin/env python3
"""Самопроверка проекта: запускать ПЕРВОЙ, до colcon build.

Ловит опечатки, битые YAML и ошибки в математике за пару секунд,
не поднимая ROS. Требует только numpy и PyYAML.

    python3 selfcheck.py
"""

import math
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "packages", "robomarvel_sim")
MODULES = os.path.join(PKG, "robomarvel_sim")

failures = []


def check(name, fn):
    try:
        fn()
        print(f"  OK   {name}")
    except Exception as exc:  # noqa: BLE001
        print(f"  FAIL {name}: {exc}")
        failures.append(name)


def syntax_ok(path):
    """Только разбор исходника: никаких .pyc, никакой записи на диск."""
    with open(path, "r", encoding="utf-8") as fh:
        compile(fh.read(), path, "exec")


print("== 1. Синтаксис Python ==")
for root, _dirs, files in os.walk(PKG):
    if "__pycache__" in root:
        continue
    for f in sorted(files):
        if f.endswith(".py"):
            path = os.path.join(root, f)
            check(os.path.relpath(path, HERE), lambda p=path: syntax_ok(p))

print("\n== 2. Метаданные пакета ==")
check("package.xml", lambda: ET.parse(os.path.join(PKG, "package.xml")))
check("urdf/robot.urdf", lambda: ET.parse(os.path.join(PKG, "urdf", "robot.urdf")))


def check_urdf_frames():
    tree = ET.parse(os.path.join(PKG, "urdf", "robot.urdf"))
    links = {l.get("name") for l in tree.getroot().findall("link")}
    for required in ("base_link", "lidar", "camera_link", "camera_optical_frame"):
        assert required in links, f"нет link '{required}'"
    for j in tree.getroot().findall("joint"):
        if j.get("name") == "lidar_joint":
            o = j.find("origin")
            assert o.get("xyz").split()[0] == "0.06", "смещение лидара по X должно быть 0.06"
            assert o.get("rpy").split()[2] == "1.57", "поворот лидара по Z должен быть 1.57"
            return
    raise AssertionError("нет lidar_joint")


check("URDF: кадры и смещение лидара", check_urdf_frames)

print("\n== 3. YAML ==")
try:
    import yaml
except ImportError:
    print("  SKIP PyYAML не установлен")
    yaml = None

if yaml:
    def load(path):
        with open(path, "r", encoding="utf-8") as fh:
            return yaml.safe_load(fh)

    nav2 = os.path.join(PKG, "config", "nav2.yaml")
    check("config/nav2.yaml", lambda: load(nav2))

    def check_nav2_values():
        d = load(nav2)
        assert d["bt_navigator"]["ros__parameters"]["odom_topic"] == "/icp/odom"
        lc = d["local_costmap"]["local_costmap"]["ros__parameters"]
        assert lc["robot_radius"] == 0.08 and lc["resolution"] == 0.025
        assert "slam_toolbox" not in d, "секция slam_toolbox должна быть убрана"

    check("nav2.yaml: значения совпадают с роботом", check_nav2_values)

    worlds = os.path.join(PKG, "config", "worlds")
    for f in sorted(os.listdir(worlds)):
        if f.endswith(".yaml"):
            check(f"config/worlds/{f}", lambda p=os.path.join(worlds, f): load(p))

print("\n== 4. Математика ==")
sys.path.insert(0, PKG)
try:
    import numpy as np

    from robomarvel_sim import kinematics as kin
    from robomarvel_sim.world import World
except Exception as exc:  # noqa: BLE001
    print(f"  FAIL импорт модулей: {exc}")
    failures.append("import")
    np = None

if np is not None:
    def check_kinematics():
        # потолок скорости: cmd 1.0 м/с -> gain 2.0 -> clamp 0.3
        assert kin.shape_command(1.0, 0.0) == (0.3, 0.0)
        # мёртвая зона по угловой
        assert kin.shape_command(0.2, 0.05)[1] == 0.0
        # доводка разворота на месте
        assert abs(kin.shape_command(0.0, 0.1)[1] - 0.8) < 1e-9
        # разгон до полной скорости ~3 с при MAX_ACCEL_LINEAR = 0.1
        v, n = 0.0, 0
        while v < kin.MAX_LINEAR - 1e-9 and n < 1000:
            v, _ = kin.ramp_body(v, 0.0, kin.MAX_LINEAR, 0.0)
            n += 1
        assert 85 <= n <= 95, f"разгон за {n} тиков"
        # нормализация не превышает предел колеса
        wl, wr = kin.body_to_wheels(kin.MAX_LINEAR, kin.MAX_ANGULAR)
        assert max(abs(wl), abs(wr)) <= kin.MAX_WHEEL_SPEED + 1e-9
        # round-trip
        v2, w2 = kin.wheels_to_body(*kin.body_to_wheels(0.2, -1.0))
        assert abs(v2 - 0.2) < 1e-9 and abs(w2 + 1.0) < 1e-9

    def check_integration():
        # езда по кругу v=0.2 w=1.0 должна замкнуться через 2*pi секунд
        x = y = yaw = 0.0
        for _ in range(int(2 * math.pi * 100)):
            x, y, yaw = kin.integrate_pose(x, y, yaw, 0.2, 1.0, 0.01)
        assert math.hypot(x, y) < 1e-3, f"невязка {math.hypot(x, y):.4f} м"

    def check_raycast():
        w = World.from_yaml(os.path.join(PKG, "config", "worlds", "classroom.yaml"))
        sx, sy, _ = w.start
        # старт не должен быть внутри препятствия
        angles = np.arange(450) / 450.0 * 2 * math.pi
        r = w.raycast(sx, sy, angles)
        assert np.isfinite(r).all(), "есть лучи, не встретившие ничего - мир не замкнут"
        assert r.min() > 0.10, f"старт слишком близко к препятствию: {r.min():.3f} м"
        assert r.max() < 12.0, f"комната больше дальности лидара: {r.max():.2f} м"
        # пересечение с отрезком: из (1.0, 0.4) вдоль +x до перегородки x=3
        assert abs(w.raycast(1.0, 0.4, np.array([0.0]))[0] - 2.0) < 1e-6, "промах по стене"
        # пересечение с окружностью: из (1.0, 1.0) вдоль +x до стула (2.4, 1.2) r=0.22
        expected = 2.4 - 1.0 - math.sqrt(0.22 ** 2 - 0.2 ** 2)
        assert abs(w.raycast(1.0, 1.0, np.array([0.0]))[0] - expected) < 1e-6, "промах по объекту"

    def check_perf():
        import time
        w = World.from_yaml(os.path.join(PKG, "config", "worlds", "classroom.yaml"))
        angles = np.arange(450) / 450.0 * 2 * math.pi
        t0 = time.time()
        for _ in range(100):
            w.raycast(1.0, 1.0, angles)
        ms = (time.time() - t0) * 10.0
        assert ms < 5.0, f"{ms:.2f} мс на скан - слишком медленно для 10 Гц"
        print(f"       ({ms:.2f} мс на скан из 450 лучей)")

    check("кинематика hwnode", check_kinematics)
    check("интегрирование позы", check_integration)
    check("raycast по classroom.yaml", check_raycast)
    check("производительность лидара", check_perf)

print()
if failures:
    print(f"ПРОВАЛЕНО: {len(failures)} - {', '.join(failures)}")
    sys.exit(1)
print("Все проверки пройдены. Можно запускать colcon build.")
