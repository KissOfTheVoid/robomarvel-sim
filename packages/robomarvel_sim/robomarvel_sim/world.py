"""Виртуальный мир: план помещения и расстояния до препятствий.

План читается из файла: стены и грани мебели становятся отрезками, столбы
и предметы - окружностями. Единственное, что умеет этот модуль, - отвечать,
через сколько метров луч из заданной точки упрётся в ближайшее препятствие.
"""

import math
import os

import numpy as np
import yaml

EPS = 1e-9


class WorldObject:
    """Предмет в помещении: человек, стул, бутылка. Он же круглое препятствие."""

    def __init__(self, cls, x, y, r, height=1.0, z=0.0):
        self.cls = cls
        self.x = float(x)
        self.y = float(y)
        self.r = float(r)
        self.height = float(height)
        self.z = float(z)


class World:
    def __init__(self, name, segments, circles, objects, start):
        self.name = name
        self.segments = np.asarray(segments, dtype=float).reshape(-1, 4)  # x1,y1,x2,y2
        self.circles = np.asarray(circles, dtype=float).reshape(-1, 3)  # x,y,r
        self.objects = objects
        self.start = start  # (x, y, yaw)

        s = self.segments
        self._ax, self._ay = s[:, 0], s[:, 1]
        self._ex, self._ey = s[:, 2] - s[:, 0], s[:, 3] - s[:, 1]

        c = self.circles
        self._cx, self._cy, self._cr = c[:, 0], c[:, 1], c[:, 2]

    @classmethod
    def from_yaml(cls, path):
        path = os.path.expanduser(path)
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)

        segments = [list(map(float, w)) for w in data.get("walls", [])]

        # каждый прямоугольник заменяем четырьмя отрезками - его сторонами
        for b in data.get("boxes", []):
            x1, y1, x2, y2 = float(b["x1"]), float(b["y1"]), float(b["x2"]), float(b["y2"])
            segments += [
                [x1, y1, x2, y1],
                [x2, y1, x2, y2],
                [x2, y2, x1, y2],
                [x1, y2, x1, y1],
            ]

        circles = []
        for c in data.get("circles", []):
            circles.append([float(c["x"]), float(c["y"]), float(c["r"])])

        objects = []
        for o in data.get("objects", []):
            obj = WorldObject(
                cls=o.get("class", "object"),
                x=o["x"],
                y=o["y"],
                r=o.get("r", 0.2),
                height=o.get("height", 1.0),
                z=o.get("z", 0.0),
            )
            objects.append(obj)
            circles.append([obj.x, obj.y, obj.r])  # предмет - ещё и круглое препятствие

        st = data.get("start", {})
        start = (float(st.get("x", 0.0)), float(st.get("y", 0.0)), float(st.get("yaw", 0.0)))

        return cls(data.get("name", os.path.basename(path)), segments, circles, objects, start)

    def raycast(self, ox, oy, angles):
        """Расстояния до ближайшего препятствия вдоль каждого луча.

        angles - углы лучей в системе координат мира, в радианах.
        Возвращает столько же чисел, сколько было углов; np.inf стоит там,
        где луч не встретил ничего.
        """
        angles = np.asarray(angles, dtype=float)
        dx = np.cos(angles)
        dy = np.sin(angles)

        best = np.full(angles.shape, np.inf)

        if self.segments.size:
            # O + t*D = A + u*E
            denom = dx[:, None] * self._ey[None, :] - dy[:, None] * self._ex[None, :]
            qx = self._ax - ox
            qy = self._ay - oy
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (qx * self._ey - qy * self._ex) / denom
                u = (qx * dy[:, None] - qy * dx[:, None]) / denom
            ok = (np.abs(denom) > EPS) & (t > EPS) & (u >= 0.0) & (u <= 1.0)
            best = np.minimum(best, np.where(ok, t, np.inf).min(axis=1))

        if self.circles.size:
            fx = ox - self._cx
            fy = oy - self._cy
            b = 2.0 * (dx[:, None] * fx[None, :] + dy[:, None] * fy[None, :])
            c = fx * fx + fy * fy - self._cr * self._cr
            disc = b * b - 4.0 * c[None, :]
            sq = np.sqrt(np.maximum(disc, 0.0))
            t1 = (-b - sq) / 2.0
            t2 = (-b + sq) / 2.0
            t = np.where(t1 > EPS, t1, np.where(t2 > EPS, t2, np.inf))
            t = np.where(disc >= 0.0, t, np.inf)
            best = np.minimum(best, t.min(axis=1))

        return best

    def range_to(self, ox, oy, angle):
        """Расстояние вдоль одного луча."""
        return float(self.raycast(ox, oy, np.array([angle]))[0])


def yaw_to_quat(yaw):
    """(x, y, z, w) для поворота вокруг Z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
