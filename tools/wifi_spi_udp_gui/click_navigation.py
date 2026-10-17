"""Click-to-go protocol and firmware-world-coordinate map controls."""
import math
import time
import dearpygui.dearpygui as dpg

NAV_FIELDS='x_cm,y_cm,nav_yaw_deg,goal_x_cm,goal_y_cm,goal_yaw_deg,distance_cm,yaw_error_deg,pos_status,nav_valid,run,position_enabled'


class ClickNavigation:
    def initialize_navigation(self):
        self.nav_mode=False
        self.nav_pose=None
        self.nav_feedback={}
        self.nav_received=0.
        self.nav_target=None
        self.nav_pending=None
        self.nav_sequence=int(time.time()*1000)&0xffffffff
        self.nav_click_start=None
        self.nav_message='未连接点选导航'

    def build_navigation_controls(self):
        with dpg.group(horizontal=True):
            dpg.add_button(label='连接点选导航 / 订阅位置回传',callback=self.connect_navigation)
            dpg.add_checkbox(label='单击地图立即前往',tag='nav_click_enabled',default_value=True)
            dpg.add_button(label='停止小车',callback=self.stop_navigation,width=self.px(110))
        with dpg.group(horizontal=True):
            dpg.add_input_double(label='目标 X / cm',tag='nav_goal_x',width=self.px(80),format='%.1f')
            dpg.add_input_double(label='目标 Y / cm',tag='nav_goal_y',width=self.px(80),format='%.1f')
            dpg.add_input_double(label='目标航向 / °',tag='nav_goal_yaw',width=self.px(80),format='%.1f')
            dpg.add_checkbox(label='保持当前航向',tag='nav_hold_yaw',default_value=True)
            dpg.add_button(label='前往输入目标',callback=self.goto_input)
        dpg.add_text('导航使用固件 Navigation/Zero 坐标，单位 cm；先在车端完成定位初始化。CSV 回放不可下发目标。',wrap=0)
        dpg.add_text(self.nav_message,tag='nav_state',color=(255,200,90))
        with dpg.handler_registry():
            dpg.add_mouse_click_handler(button=dpg.mvMouseButton_Left,callback=self.navigation_mouse_down)
            dpg.add_mouse_release_handler(button=dpg.mvMouseButton_Left,callback=self.navigation_mouse_up)

    def connect_navigation(self):
        if not self.receiver.is_running:
            self.nav_message='请先在调参页开始监听并设置模块 IP/端口'
            return
        if self.nav_pending:
            self.nav_message='请先等待当前指令确认'
            return
        self.clear_route()
        self.nav_mode=True
        self.nav_pose=None
        self.nav_received=0.
        self.nav_target=None
        dpg.set_value('mcar_names',NAV_FIELDS)
        dpg.set_value('mcar_period',50)
        self.apply_subscription()
        self.nav_message='正在订阅位置、目标、剩余距离、运行和到达状态'

    def next_navigation_id(self):
        self.nav_sequence=(self.nav_sequence+1)&0xffffffff
        return self.nav_sequence

    def navigation_endpoint(self):
        if not self.receiver.is_running:
            raise ValueError('请先开始 UDP 监听')
        ip=dpg.get_value('remote_ip').strip()
        port=int(dpg.get_value('remote_port'))
        if not ip or not 1<=port<=65535:
            raise ValueError('请设置正确的模块 IP 和端口')
        return ip,port

    def navigation_send(self,packet,endpoint):
        self.receiver.send_bound(packet,*endpoint)
        self.log(f'导航 -> {endpoint[0]}:{endpoint[1]} {packet.decode("ascii").strip()}','tx')

    def goto_input(self):
        self.goto_navigation(dpg.get_value('nav_goal_x'),dpg.get_value('nav_goal_y'))

    def goto_navigation(self,x,y):
        try:
            endpoint=self.navigation_endpoint()
            if not self.nav_mode or self.route_csv is not None:
                raise ValueError('请先连接点选导航，CSV / 里程重建视图不可下发目标')
            if self.subscription.pending or self.subscription.error:
                raise ValueError('位置订阅尚未成功，请等待设备确认')
            if self.nav_pose is None or time.monotonic()-self.nav_received>.5 or not self.nav_feedback.get('nav_valid'):
                raise ValueError('定位无效或位置回传超过 0.5 秒未更新，未发送目标')
            yaw=self.nav_pose[2] if dpg.get_value('nav_hold_yaw') else float(dpg.get_value('nav_goal_yaw'))
            yaw=(yaw+180)%360-180
            x,y=float(x),float(y)
            if not all(math.isfinite(v) for v in (x,y,yaw)) or abs(x)>10000 or abs(y)>10000:
                raise ValueError('坐标须为有限数，X/Y 范围 ±10000 cm')
            x,y,yaw=(round(v,3) for v in (x,y,yaw))
            ident=self.next_navigation_id()
            packet=f'GOTO {ident} {x:.3f} {y:.3f} {yaw:.3f}\n'.encode('ascii')
            self.navigation_send(packet,endpoint)
            self.nav_target=(x,y,yaw)
            self.nav_pending=dict(id=ident,kind='NAV',packet=packet,endpoint=endpoint,deadline=time.monotonic()+1,attempts=1)
            self.nav_message=f'目标 ({x:.1f}, {y:.1f}) cm，等待设备确认 #{ident}'
            dpg.set_value('nav_goal_x',x); dpg.set_value('nav_goal_y',y)
            self.route_dirty=True
        except (ValueError,OSError) as exc:
            self.nav_message=str(exc)

    def stop_navigation(self):
        try:
            endpoint=self.nav_pending['endpoint'] if self.nav_pending else self.navigation_endpoint()
            ident=self.next_navigation_id()
            packet=f'STOP {ident}\n'.encode('ascii')
            self.navigation_send(packet,endpoint)
            self.nav_pending=dict(id=ident,kind='STOP',packet=packet,endpoint=endpoint,deadline=time.monotonic()+1,attempts=1)
            self.nav_message='停止指令已发送，等待设备确认'
            self.nav_target=None
            self.route_dirty=True
        except (ValueError,OSError) as exc:
            self.nav_message=f'停止发送失败：{exc}'

    def handle_navigation_reply(self,kind,value):
        if kind not in ('NAV','STOP','NAVERR'):
            return False
        parts=value.split()
        pending=self.nav_pending
        if not pending or int(parts[0])!=pending['id']:
            return True
        if kind=='NAVERR':
            self.nav_message='设备拒绝导航：'+' '.join(parts[1:])
            self.nav_pending=None
            self.nav_target=None
        elif kind==pending['kind']:
            if kind=='NAV':
                goal=tuple(float(v) for v in parts[1:])
                if len(goal)!=3 or any(abs(a-b)>.002 for a,b in zip(goal,self.nav_target)):
                    return True
                self.nav_message='设备已接收目标，等待运行状态回传'
            else:
                self.nav_message='设备已确认停止'
            self.nav_pending=None
        self.route_dirty=True
        return True

    def accept_navigation(self,values,timestamp=None,address=None):
        if not self.nav_mode:
            return False
        if address is not None and address!=(dpg.get_value('remote_ip').strip(),int(dpg.get_value('remote_port'))):
            return True
        required=('x_cm','y_cm','nav_yaw_deg','nav_valid','pos_status','run','goal_x_cm','goal_y_cm','goal_yaw_deg','distance_cm')
        if not all(key in values for key in required):
            return True
        try:
            data={k:float(v) for k,v in values.items()}
            if not all(math.isfinite(data[k]) for k in required):
                return True
        except (ValueError,TypeError):
            return True
        self.nav_feedback=data
        self.nav_received=time.monotonic()-max(0.,time.time()-timestamp) if timestamp is not None else time.monotonic()
        self.nav_pose=(data['x_cm'],data['y_cm'],data['nav_yaw_deg'])
        if self.route_csv is None and data['nav_valid']:
            point=self.nav_pose[:2]
            if self.route_live:
                self.route_length+=math.hypot(point[0]-self.route_live[-1][0],point[1]-self.route_live[-1][1])
            self.route_live.append(point)
            self.route_count+=1
            self.route_dirty=True
        return True

    def poll_navigation(self):
        pending=self.nav_pending
        if pending and time.monotonic()>=pending['deadline']:
            if pending['attempts']>=3:
                self.nav_message='设备确认超时，执行状态未知；可点击停止小车。请确认已烧录点选导航固件。'
                self.nav_pending=None
            else:
                try:
                    self.navigation_send(pending['packet'],pending['endpoint'])
                    pending['attempts']+=1; pending['deadline']=time.monotonic()+1
                except (ValueError,OSError) as exc:
                    self.nav_message=f'重发失败：{exc}'; self.nav_pending=None
        text=self.nav_message
        if self.nav_mode and self.subscription.error:
            text='位置订阅失败：'+self.subscription.error
        if self.nav_mode and self.nav_pose:
            x,y,yaw=self.nav_pose
            data=self.nav_feedback
            state={-3:'航向停滞',-2:'参数错误',-1:'定位无效',0:'待机',1:'行驶中',2:'到达稳定中',3:'已到达'}.get(int(data.get('pos_status',0)),'未知')
            if self.nav_target:
                actual=tuple(data.get(k) for k in ('goal_x_cm','goal_y_cm','goal_yaw_deg'))
                if any(v is None or abs(v-t)>.02 for v,t in zip(actual,self.nav_target)):
                    state='等待新目标生效' if self.nav_pending else '设备目标与地图目标不一致'
            if time.monotonic()-self.nav_received>.5:
                state='位置回传中断，禁止新目标'
            elif not data.get('nav_valid'):
                state='定位无效，禁止新目标'
            elif self.nav_target and not self.nav_pending:
                goal=(data.get('goal_x_cm'),data.get('goal_y_cm'),data.get('goal_yaw_deg'))
                if all(v is not None and abs(v-t)<.02 for v,t in zip(goal,self.nav_target)):
                    text+=f' | 剩余 {data.get("distance_cm",0):.1f} cm | Run={int(data.get("run",0))}'
            text+=f' | {state} | 车位 ({x:.1f}, {y:.1f}) cm，航向 {yaw:.1f}°'
        dpg.set_value('nav_state',text)

    def navigation_mouse_down(self):
        self.nav_click_start=None
        if self.active_page=='route' and dpg.is_item_hovered('route_plot') and dpg.get_value('nav_click_enabled'):
            self.nav_click_start=(dpg.get_mouse_pos(local=False),dpg.get_plot_mouse_pos())

    def navigation_mouse_up(self):
        start=self.nav_click_start
        self.nav_click_start=None
        if start and self.active_page=='route' and dpg.is_item_hovered('route_plot'):
            p=dpg.get_mouse_pos(local=False)
            if math.hypot(p[0]-start[0][0],p[1]-start[0][1])<5:
                self.goto_navigation(*start[1])

    def draw_navigation(self):
        if not self.nav_mode or self.route_csv is not None:
            return
        points=list(self.route_live[::max(1,len(self.route_live)//1000)])
        if self.nav_pose:
            x,y,yaw=self.nav_pose
            dpg.add_scatter_series([x],[y],label='小车当前位置',parent='route_y')
            angle=math.radians(yaw)
            dpg.add_line_series([x,x+15*math.cos(angle)],[y,y+15*math.sin(angle)],label='车头',parent='route_y')
            points.append((x,y))
        if self.nav_target:
            x,y,_=self.nav_target
            dpg.add_scatter_series([x],[y],label='导航目标',parent='route_y')
            points.append((x,y))
        if dpg.get_value('route_fit'):
            xs=[0.]+[p[0] for p in points]; ys=[0.]+[p[1] for p in points]
            lo_x,hi_x=min(xs),max(xs); lo_y,hi_y=min(ys),max(ys)
            span=max(200.,hi_x-lo_x,hi_y-lo_y)*.6
            cx=(hi_x+lo_x)/2; cy=(hi_y+lo_y)/2
            dpg.set_axis_limits('route_x',cx-span,cx+span)
            dpg.set_axis_limits('route_y',cy-span,cy+span)
        else:
            dpg.set_axis_limits_auto('route_x'); dpg.set_axis_limits_auto('route_y')
