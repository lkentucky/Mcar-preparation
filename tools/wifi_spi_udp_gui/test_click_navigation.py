"""Real DearPyGui window and loopback UDP navigation integration."""
import ctypes
import ctypes.wintypes
import socket
import struct
import tempfile
import time
from pathlib import Path
import dearpygui.dearpygui as dpg
from test_slider_gui import open_monitor
from click_navigation import NAV_FIELDS
from mcar_telemetry import control_reply


def frames(m,count=5):
    for _ in range(count):
        dpg.run_callbacks(dpg.get_callback_queue()); m.refresh(); dpg.render_dearpygui_frame(); time.sleep(.02)


def run():
    for raw in (b'MCAR NAV bad 1 2 3\n',b'MCAR NAV 1 nan 2 3\n',b'MCAR STOP 1 extra\n'):
        try:
            control_reply(raw)
            raise AssertionError('invalid reply accepted')
        except ValueError:
            pass
    with tempfile.TemporaryDirectory() as tmp, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as device:
        device.bind(('127.0.0.1',0)); device.settimeout(1)
        m=open_monitor(Path(tmp)/'workspace.json')
        try:
            dpg.set_value('bind_ip','127.0.0.1'); dpg.set_value('bind_port',0)
            dpg.set_value('remote_ip','127.0.0.1'); dpg.set_value('remote_port',device.getsockname()[1])
            m.start_listening(); m.switch_page('route'); m.connect_navigation()
            expected=[b'STREAM 0\n',b'SUB '+NAV_FIELDS.encode()+b'\n',b'RATE 50\n',b'STREAM 1\n']
            for command in expected:
                data,client=device.recvfrom(4096); assert data==command,(data,command)
                device.sendto(b'MCAR '+data,client); frames(m)
            assert not m.subscription.pending and m.nav_mode
            values=[20,-15,35,0,0,0,20,0,0,1,0,1]
            def telemetry():
                device.sendto(struct.pack('<12f',*values)+b'\0\0\x80\x7f',client); frames(m,3)
            telemetry()
            assert m.route_live[-1]==(20,-15),m.route_live
            # Feed the plot-coordinate click handlers; native input is unavailable
            # in the tool desktop. Networking and window rendering remain real.
            from unittest.mock import patch
            with patch.object(dpg,'is_item_hovered',return_value=True), patch.object(dpg,'get_mouse_pos',return_value=(400,400)), patch.object(dpg,'get_plot_mouse_pos',return_value=(50,-25)):
                m.navigation_mouse_down(); m.navigation_mouse_up()
            data,_=device.recvfrom(4096)
            tokens=data.decode().split(); assert tokens[0]=='GOTO',data
            ident=int(tokens[1]); x,y,yaw=map(float,tokens[2:]); assert yaw==35
            device.sendto(f'MCAR NAV {ident} {x} {y} {yaw}\n'.encode(),client); frames(m)
            assert m.nav_pending is None
            values[:3]=[x,y,yaw]; values[3:9]=[x,y,yaw,0,0,3]
            telemetry(); assert '已到达' in dpg.get_value('nav_state'),dpg.get_value('nav_state')
            # Stale feedback cannot generate a new command.
            m.nav_received-=2; previous=m.nav_sequence; m.goto_navigation(10,20); assert m.nav_sequence==previous
            # Same packet/id on retry, with STOP superseding pending navigation.
            telemetry(); m.goto_navigation(30,40); packet,_=device.recvfrom(4096)
            m.nav_pending['deadline']=0; m.poll_navigation(); repeated,_=device.recvfrom(4096); assert packet==repeated
            m.stop_navigation(); stop,_=device.recvfrom(4096); stop_id=int(stop.split()[1])
            assert stop.startswith(b'STOP ') and stop_id>int(packet.split()[1])
            device.sendto(f'MCAR STOP {stop_id}\n'.encode(),client); frames(m); assert m.nav_pending is None
            m.route_csv='playback.csv'; previous=m.nav_sequence; m.goto_navigation(0,0); assert m.nav_sequence==previous
            print('PASS: subscription handshake, absolute pose, plot click handlers, UDP goal/ACK, arrival, stale block, retry, STOP, CSV block')
        finally:
            m.receiver.stop(); dpg.destroy_context()

if __name__=='__main__':
    run()
