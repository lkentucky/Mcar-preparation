"""读取第三方 CC0 OBJ 飞机网格，转换为上位机的机体坐标系。"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math
from pathlib import Path
import sys


Point3 = tuple[float, float, float]


@dataclass(frozen=True)
class MeshFace:
    indices: tuple[int, ...]
    center: Point3
    normal: Point3


@dataclass(frozen=True)
class AircraftMesh:
    vertices: tuple[Point3, ...]
    faces: tuple[MeshFace, ...]


def _resource_path(name: str) -> Path:
    """源码运行和 PyInstaller 单文件运行都能找到打包的模型。"""
    root = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
    return root / name


def _cross(a: Point3, b: Point3) -> Point3:
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


@lru_cache(maxsize=2)
def load_aircraft_mesh(quality: str = "high") -> AircraftMesh:
    """OBJ 只在首次使用时解析一次；法线和面中心预计算，收包热路径不读磁盘。"""
    if quality not in ("high", "low"):
        raise ValueError(f"不支持的模型质量：{quality}")
    path = _resource_path(f"aircraft_{quality}.obj")
    raw_vertices: list[Point3] = []
    raw_faces: list[tuple[int, ...]] = []
    with path.open("r", encoding="utf-8", errors="replace") as file:
        for line in file:
            if line.startswith("v "):
                _, x, y, z, *_ = line.split()
                raw_vertices.append((float(x), float(y), float(z)))
            elif line.startswith("f "):
                tokens = line.split()[1:]
                indices = tuple(int(token.split("/")[0]) - 1 for token in tokens)
                if len(indices) >= 3:
                    raw_faces.append(indices)
    if not raw_vertices or not raw_faces:
        raise ValueError(f"OBJ 没有有效的顶点或面：{path}")

    # 原素材：X 为左右翼、Y 为竖直、Z 正向为机头；转换为 +X 前、+Y 右、+Z 上。
    mins = [min(p[i] for p in raw_vertices) for i in range(3)]
    maxs = [max(p[i] for p in raw_vertices) for i in range(3)]
    middle = [(a + b) / 2 for a, b in zip(mins, maxs)]
    scale = 215.0 / (maxs[0] - mins[0])
    vertices: tuple[Point3, ...] = tuple(((p[2] - middle[2]) * scale,
                                           (p[0] - middle[0]) * scale,
                                           (p[1] - middle[1]) * scale) for p in raw_vertices)
    faces: list[MeshFace] = []
    for indices in raw_faces:
        a, b, c = (vertices[i] for i in indices[:3])
        ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
        normal = _cross(ab, ac)
        norm = math.sqrt(sum(v * v for v in normal))
        if norm < 1e-8:
            continue
        normal = tuple(value / norm for value in normal)  # type: ignore[assignment]
        center = tuple(sum(vertices[i][axis] for i in indices) / len(indices)
                       for axis in range(3))
        faces.append(MeshFace(indices, center, normal))  # type: ignore[arg-type]
    return AircraftMesh(vertices, tuple(faces))
