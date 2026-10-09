"""使用 DearPyGui 矢量绘图绘制 IMU 机体姿态，不依赖额外 3D 库。"""

from __future__ import annotations

import math
from typing import Sequence

import dearpygui.dearpygui as dpg

from aircraft_model import load_aircraft_mesh


Point3 = tuple[float, float, float]
Color = tuple[int, int, int, int]


def _rotate(point: Sequence[float], roll: float, pitch: float, yaw: float) -> Point3:
    """采用 Rz(yaw) * Ry(pitch) * Rx(roll) 欧拉角顺序。"""
    x, y, z = point
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        cy * cp * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z,
        sy * cp * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z,
        -sp * x + cp * sr * y + cp * cr * z,
    )


def _unit(point: Point3) -> Point3:
    length = math.sqrt(sum(value * value for value in point)) or 1.0
    return tuple(value / length for value in point)  # type: ignore[return-value]


def _cross(a: Point3, b: Point3) -> Point3:
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def _draw_scene(drawlist_tag: str, width: float, height: float, roll: float, pitch: float, yaw: float,
                view_yaw: float, view_elev: float, zoom: float, ui_scale: float,
                quality: str) -> None:
    """固定世界参考网格，按深度排序绘制实体机体。"""
    bg: Color = (13, 20, 31, 255)
    grid: Color = (31, 49, 66, 255)
    dpg.draw_rectangle((0, 0), (width, height), color=bg, fill=bg, parent=drawlist_tag)

    azimuth, elevation = math.radians(view_yaw), math.radians(view_elev)
    camera = _unit((math.cos(elevation) * math.cos(azimuth),
                    math.cos(elevation) * math.sin(azimuth), math.sin(elevation)))
    right = _unit(_cross((0.0, 0.0, 1.0), camera))
    up = _unit(_cross(camera, right))
    scale = min(width / 420.0, height / 350.0) * 1.5 * zoom
    center_x, center_y = width / 2.0, height / 2.0 + height * 0.04

    def project(point: Sequence[float]) -> tuple[float, float]:
        return (center_x + _dot(point, right) * scale,
                center_y - _dot(point, up) * scale)

    # 低亮度地面网格提供稳定的空间参照，IMU 旋转时网格不随之转动。
    floor_z = -105.0
    for distance in range(-360, 361, 60):
        dpg.draw_line(project((-360, distance, floor_z)), project((360, distance, floor_z)),
                      color=grid, thickness=1, parent=drawlist_tag)
        dpg.draw_line(project((distance, -360, floor_z)), project((distance, 360, floor_z)),
                      color=grid, thickness=1, parent=drawlist_tag)
    dpg.draw_line(project((-360, 0, floor_z)), project((360, 0, floor_z)),
                  color=(83, 60, 66, 255), thickness=1.3, parent=drawlist_tag)
    dpg.draw_line(project((0, -360, floor_z)), project((0, 360, floor_z)),
                  color=(51, 89, 68, 255), thickness=1.3, parent=drawlist_tag)

    # 导入真实 OBJ 顶点。法线和面中心在加载时预计算，每帧仅旋转顶点并画可见面。
    mesh = load_aircraft_mesh(quality)
    transformed = [_rotate(point, roll, pitch, yaw) for point in mesh.vertices]
    projected = [project(point) for point in transformed]
    light = _unit((0.40, -0.40, 0.82))
    visible_faces: list[tuple[float, tuple[int, ...], Color]] = []
    for face in mesh.faces:
        normal = _rotate(face.normal, roll, pitch, yaw)
        if _dot(normal, camera) <= 0.02:
            continue  # 背面裁剪：减少绘图调用，避免机体背面穿透前表面。
        depth = _dot(_rotate(face.center, roll, pitch, yaw), camera)
        x, y, z = face.center
        if x > 65:
            base = (116, 155, 184)  # 机头采用较深的金属色，避免卡通感
        elif z > 15 and -25 < x < 45 and abs(y) < 27:
            base = (49, 103, 143)  # 座舱玻璃
        elif abs(y) > 82:
            base = (66, 147, 192)  # 翼尖颜色区别于机身
        elif z < -14:
            base = (96, 125, 151)
        else:
            base = (168, 196, 216)
        brightness = 0.52 + 0.48 * max(0.0, _dot(normal, light))
        color: Color = tuple(min(255, round(component * brightness)) for component in base) + (255,)
        visible_faces.append((depth, face.indices, color))
    # 远面先画、近面覆盖；对于此轻量模型，排序比完整逐像素深度缓冲更省资源。
    for _, indices, color in sorted(visible_faces, key=lambda item: item[0]):
        dpg.draw_polygon([projected[index] for index in indices], color=color, fill=color,
                         thickness=1.0, parent=drawlist_tag)

    # 机头、左右翼端颜色标识及机体系三轴，使大角度翻转也不会认错方向。
    labels = [((88, 0, 0), (255, 185, 78, 255), "FRONT"),
              ((0, -108, 0), (244, 87, 87, 255), "L"),
              ((0, 108, 0), (90, 228, 130, 255), "R")]
    text_size = round(15 * ui_scale)
    for point, color, label in labels:
        x, y = project(_rotate(point, roll, pitch, yaw))
        dpg.draw_circle((x, y), max(4, 4 * ui_scale), color=color, fill=color, parent=drawlist_tag)
        dpg.draw_text((x + 9 * ui_scale, y - 8 * ui_scale), label, color=color,
                      size=text_size, parent=drawlist_tag)

    # 小型坐标罗盘放在右下角，不再让 XYZ 线穿过机身干扰姿态辨认。
    origin = (width - 92 * ui_scale, height - 92 * ui_scale)
    dpg.draw_circle(origin, 31 * ui_scale, color=(56, 77, 98, 255),
                    fill=(18, 29, 43, 235), parent=drawlist_tag)
    for direction, color, label in [((65, 0, 0), (245, 104, 104, 255), "X"),
                                    ((0, 65, 0), (94, 223, 135, 255), "Y"),
                                    ((0, 0, 65), (113, 165, 255, 255), "Z")]:
        rotated = _rotate(direction, roll, pitch, yaw)
        endpoint = (origin[0] + _dot(rotated, right) * 0.48 * ui_scale,
                    origin[1] - _dot(rotated, up) * 0.48 * ui_scale)
        dpg.draw_line(origin, endpoint, color=color, thickness=2 * ui_scale, parent=drawlist_tag)
        dpg.draw_circle(endpoint, 3 * ui_scale, color=color, fill=color, parent=drawlist_tag)
        dpg.draw_text((endpoint[0] + 5 * ui_scale, endpoint[1] - 5 * ui_scale), label,
                      color=color, size=text_size, parent=drawlist_tag)

    dpg.draw_text((18 * ui_scale, 16 * ui_scale), "IMU  /  ATTITUDE",
                  color=(204, 226, 243, 255), size=round(17 * ui_scale), parent=drawlist_tag)
    dpg.draw_text((18 * ui_scale, 43 * ui_scale), "ZYX Euler  /  degrees",
                  color=(115, 147, 173, 255), size=round(13 * ui_scale), parent=drawlist_tag)
    dpg.draw_text((18 * ui_scale, height - 28 * ui_scale),
                  f"ROLL {math.degrees(roll):+06.1f}°     PITCH {math.degrees(pitch):+06.1f}°     YAW {math.degrees(yaw):+06.1f}°",
                  color=(177, 205, 226, 255), size=round(14 * ui_scale), parent=drawlist_tag)


def draw_aircraft_attitude(drawlist_tag: str, roll_deg: float, pitch_deg: float, yaw_deg: float,
                           view_yaw: float = -55.0, view_elev: float = 30.0,
                           zoom: float = 1.0, ui_scale: float = 1.0,
                           quality: str = "high") -> None:
    """根据 IMU 欧拉角重绘导入的飞机模型。"""
    try:
        width, height = dpg.get_item_rect_size(drawlist_tag)
    except SystemError:
        width, height = 0, 0
    width, height = max(430 * ui_scale, width), max(360 * ui_scale, height)
    dpg.delete_item(drawlist_tag, children_only=True)
    angles = tuple(map(math.radians, (roll_deg, pitch_deg, yaw_deg)))
    _draw_scene(drawlist_tag, width, height, *angles, view_yaw, view_elev, zoom, ui_scale, quality)
