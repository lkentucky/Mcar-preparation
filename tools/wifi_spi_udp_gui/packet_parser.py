"""可配置的二进制 UDP 数据包解析器。

本模块不依赖 GUI，因而可单独导入、测试，或替换为项目中的专用协议解析器。
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Any, Iterable


# 类型名 -> struct 格式字符。逐飞 WiFi-SPI2.0 常见 MCU 数据为小端序。
TYPE_FORMATS: dict[str, str] = {
    "float32": "f",
    "float64": "d",
    "int8": "b",
    "uint8": "B",
    "int16": "h",
    "uint16": "H",
    "int32": "i",
    "uint32": "I",
    "int64": "q",
    "uint64": "Q",
}


@dataclass(frozen=True)
class FieldRule:
    """一个数据字段的解析规则，offset 单位为字节。"""

    name: str
    data_type: str
    offset: int
    byte_order: str = "little"

    @property
    def size(self) -> int:
        """字段占用的字节数。"""
        if self.data_type not in TYPE_FORMATS:
            raise ValueError(f"不支持的数据类型：{self.data_type}")
        return struct.calcsize(TYPE_FORMATS[self.data_type])


def parse_packet(packet: bytes, rules: Iterable[FieldRule]) -> tuple[dict[str, Any], dict[str, str]]:
    """按规则解析一个二进制 UDP 数据包。

    参数:
        packet: 原始 UDP 负载。
        rules: 字段规则的可迭代对象。

    返回:
        ``(values, errors)``。单个字段出错不会影响其余字段的解析：
        ``values`` 是成功字段和值，``errors`` 的键是字段名、值是错误原因。
    """
    values: dict[str, Any] = {}
    errors: dict[str, str] = {}

    for rule in rules:
        field_name = rule.name.strip() or "未命名字段"
        try:
            if rule.offset < 0:
                raise ValueError("起始字节不能为负数")
            if rule.byte_order not in ("little", "big"):
                raise ValueError("字节序只能是 little 或 big")
            fmt = TYPE_FORMATS[rule.data_type]
            prefix = "<" if rule.byte_order == "little" else ">"
            size = struct.calcsize(fmt)
            if rule.offset + size > len(packet):
                raise ValueError(f"数据包长度 {len(packet)} 字节，不足以读取 {rule.offset}:{rule.offset + size}")
            values[field_name] = struct.unpack_from(prefix + fmt, packet, rule.offset)[0]
        except (ValueError, KeyError, struct.error) as exc:
            errors[field_name] = str(exc)

    return values, errors
