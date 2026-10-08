"""独立 UDP 接收线程。该模块绝不直接调用 DearPyGui API。"""

from __future__ import annotations

import queue
import socket
import threading
import time
from typing import Literal


EventKind = Literal["packet", "error", "status"]


class UdpReceiver:
    """以后台线程接收 UDP 数据，并通过线程安全队列交给 GUI 主线程。"""

    def __init__(self, queue_size: int = 4096) -> None:
        self.events: queue.Queue[tuple[EventKind, object]] = queue.Queue(maxsize=queue_size)
        self._running = threading.Event()
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._state_lock = threading.Lock()
        self._status = "未监听"
        self.received_count = 0
        self.dropped_count = 0

    @property
    def is_running(self) -> bool:
        return self._running.is_set()

    @property
    def status(self) -> str:
        with self._state_lock:
            return self._status

    def _set_status(self, text: str) -> None:
        with self._state_lock:
            self._status = text

    def _put_event(self, kind: EventKind, payload: object) -> None:
        """队列满时宁可丢弃新帧，也不能让接收线程阻塞。"""
        try:
            self.events.put_nowait((kind, payload))
        except queue.Full:
            self.dropped_count += 1

    def start(self, bind_ip: str, bind_port: int) -> None:
        """绑定本机地址并启动接收线程；地址无效会抛出 OSError。"""
        if self.is_running:
            raise RuntimeError("UDP 已在监听")
        if not 0 <= bind_port <= 65535:
            raise ValueError("监听端口必须在 0~65535 之间")

        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_socket.settimeout(0.25)  # stop() 最多等待一个短超时周期
        try:
            udp_socket.bind((bind_ip, bind_port))
        except OSError:
            udp_socket.close()
            raise

        self.received_count = 0
        self.dropped_count = 0
        self._socket = udp_socket
        self._running.set()
        self._set_status(f"监听中：{bind_ip}:{bind_port}")
        self._thread = threading.Thread(target=self._receive_loop, name="udp-receiver", daemon=True)
        self._thread.start()
        self._put_event("status", self.status)

    def _receive_loop(self) -> None:
        """后台阻塞接收循环。所有异常都转成事件，不让线程异常退出拖垮 GUI。"""
        while self._running.is_set():
            try:
                assert self._socket is not None
                data, address = self._socket.recvfrom(65535)
                received_at = time.time()
                self.received_count += 1
                self._put_event("packet", (received_at, data, address))
            except socket.timeout:
                continue
            except OSError as exc:
                # stop() 关闭 socket 会触发 OSError，属于预期路径。
                if self._running.is_set():
                    self._set_status("网络异常，已停止监听")
                    self._put_event("error", f"UDP 接收异常：{exc}")
                break
            except Exception as exc:  # 防御性保护，线程不可因坏包崩溃
                self._put_event("error", f"接收线程未知异常：{exc}")

        self._running.clear()

    def stop(self) -> None:
        """停止接收线程并释放端口，可重复调用。"""
        self._running.clear()
        sock, self._socket = self._socket, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
        self._set_status("未监听")
        self._put_event("status", self.status)

    @staticmethod
    def send(data: bytes, remote_ip: str, remote_port: int) -> int:
        """发送一个 UDP 数据报，返回实际发送字节数。"""
        if not 1 <= remote_port <= 65535:
            raise ValueError("目标端口必须在 1~65535 之间")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            return sender.sendto(data, (remote_ip, remote_port))

    def send_bound(self, data: bytes, remote_ip: str, remote_port: int) -> int:
        """复用监听 socket，让命令源端口与模块配置的电脑端口一致。

        接收线程只 recvfrom，GUI 线程 sendto；发送异常交由 GUI 处理。
        不使用临时源端口，兼容限制固定 UDP 对端的模块固件。
        """
        if not 1 <= remote_port <= 65535:
            raise ValueError("目标端口必须在 1~65535 之间")
        sock = self._socket
        if not self.is_running or sock is None:
            raise OSError("请先开始监听，以复用监听端口发送指令")
        return sock.sendto(data, (remote_ip, remote_port))
