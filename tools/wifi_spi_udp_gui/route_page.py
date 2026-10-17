"""Route view using the supplied odometry reconstruction engine."""
import csv
import math
import time
from pathlib import Path
from types import SimpleNamespace
import dearpygui.dearpygui as dpg
from plot_mcar_path import load_csv, reconstruct, find_motion_bouts, choose_bouts


from click_navigation import ClickNavigation


class RoutePage(ClickNavigation):
    def initialize_route(self):
        self.initialize_navigation()
        self.route_args = SimpleNamespace(dt_ms=10., max_gap_ms=100., scale_x=.97,
            scale_y=1.04, lateral_factor=.901589, diag_comp=1., lateral_coupling=0.,
            split_gap_s=.5, move_threshold=3., segment='auto')
        self.route_live = []
        self.route_tracks = []
        self.route_previous = None
        self.route_yaw0 = None
        self.route_length = 0.
        self.route_count = 0
        self.route_last_draw = 0.
        self.route_dirty = True
        self.route_csv = None

    def build_route_page(self):
        dpg.add_text('路线图 / 地图点选导航', color=(98,185,255))
        self.build_navigation_controls()
        dpg.add_text('轮速重建模式起点归零；导航模式使用固件 Zero 原点。前进 +X，左移 +Y。')
        with dpg.group(horizontal=True):
            dpg.add_button(label='订阅路线所需变量', callback=self.subscribe_route)
            dpg.add_button(label='清空并开始实时路线', callback=self.clear_route)
            dpg.add_checkbox(label='自动适应', tag='route_fit', default_value=True)
            dpg.add_button(label='导出路线 CSV', callback=self.export_route)
        with dpg.group(horizontal=True):
            dpg.add_input_text(tag='route_file', hint='CSV 文件路径', width=self.px(460))
            dpg.add_button(label='导入 CSV', callback=self.import_route)
            dpg.add_input_text(label='运动段 auto / all / 编号', tag='route_segment', default_value='auto', width=self.px(80))
        with dpg.group(horizontal=True):
            for key, label in [('dt_ms','采样 ms'),('scale_x','前进比例'),('scale_y','横移比例'),('lateral_factor','横移系数')]:
                dpg.add_input_double(label=label, tag='route_'+key, default_value=getattr(self.route_args,key), width=self.px(85), format='%.6g')
            dpg.add_button(label='应用参数并重算', callback=self.apply_route_parameters)
        dpg.add_text('实时及无时间戳 CSV 默认 10 ms；请与设备上传周期一致。时间戳间隔超过 100 ms 跳过。', color=(165,179,196))
        dpg.add_text('等待 nav_yaw_deg 和 ul/ur/dl/dr_cmps 数据', tag='route_status')
        with dpg.plot(tag='route_plot', width=-1, height=-self.px(45), equal_aspects=True, anti_aliased=True):
            dpg.add_plot_legend()
            dpg.add_plot_axis(dpg.mvXAxis,label='X / cm',tag='route_x')
            dpg.add_plot_axis(dpg.mvYAxis,label='Y / cm',tag='route_y')
        dpg.add_input_text(label='导出文件', tag='route_output', default_value='mcar_route.csv', width=self.px(460))

    def subscribe_route(self):
        self.nav_mode=False
        dpg.set_value('mcar_names','nav_yaw_deg,ul_cmps,ur_cmps,dl_cmps,dr_cmps')
        dpg.set_value('mcar_period', max(2,round(self.route_args.dt_ms)))
        self.clear_route()
        self.apply_subscription()

    def clear_route(self):
        self.route_csv = None
        self.route_live = []
        self.route_tracks = []
        self.route_previous = None
        self.route_yaw0 = None
        self.route_length = 0.
        self.route_count = 0
        self.route_dirty = True

    def accept_route(self, values, timestamp=None, address=None):
        if self.accept_navigation(values, timestamp, address):
            return
        if self.route_csv is not None:
            return
        row = {str(k).lower(): v for k,v in values.items()}
        try:
            sample = dict(time_s=None, yaw_deg=float(row['nav_yaw_deg']), wheels=[float(row[w+'_cmps']) for w in ('ul','ur','dl','dr')])
            if not all(math.isfinite(v) for v in [sample['yaw_deg']]+sample['wheels']):
                return
        except (KeyError,ValueError,TypeError):
            self.route_previous = None
            return
        if self.route_yaw0 is None:
            self.route_yaw0 = sample['yaw_deg']
            self.route_live.append((0.,0.))
        if self.route_previous is not None:
            x,y,stats = reconstruct([self.route_previous,sample],self.route_args)
            theta=math.radians(self.route_previous['yaw_deg']-self.route_yaw0)
            dx=x[-1]*math.cos(theta)-y[-1]*math.sin(theta)
            dy=x[-1]*math.sin(theta)+y[-1]*math.cos(theta)
            px,py=self.route_live[-1]
            self.route_live.append((px+dx,py+dy))
            self.route_length += stats['path_cm']
        self.route_previous=sample
        self.route_count += 1
        self.route_tracks=[dict(label='固件定位轨迹' if self.nav_mode else '实时路线', cross=None, along=None)]
        self.route_dirty=True

    def import_route(self):
        try:
            path=dpg.get_value('route_file').strip().strip('"')
            samples, yaw, clock=load_csv(path)
            self.route_args.segment=dpg.get_value('route_segment').strip()
            bouts=find_motion_bouts(samples,self.route_args)
            selected=choose_bouts(samples,bouts,self.route_args,path)
            tracks=[]
            length=0.
            for a,b,n in selected:
                x,y,stats=reconstruct(samples[a:b+1],self.route_args)
                tracks.append(dict(label=f'{Path(path).stem} #{n or 1}',cross=x,along=y))
                length+=stats['path_cm']
            self.route_tracks=tracks
            self.route_csv=path
            self.route_length=length
            self.route_count=sum(len(t['cross']) for t in tracks)
            self.route_dirty=True
            self.refresh_route(force=True)
        except (OSError,ValueError,csv.Error) as exc:
            dpg.set_value('route_status',f'导入失败：{exc}')

    def apply_route_parameters(self):
        try:
            params={k:float(dpg.get_value('route_'+k)) for k in ('dt_ms','scale_x','scale_y','lateral_factor')}
            if any(not math.isfinite(v) or v<=0 for v in params.values()):
                raise ValueError('参数必须是大于零的有限数值')
            for k,v in params.items():
                setattr(self.route_args,k,v)
            if self.route_csv is not None:
                self.import_route()
            else:
                self.clear_route()
                dpg.set_value('route_status','参数已应用，实时路线已重新归零')
        except ValueError as exc:
            dpg.set_value('route_status',str(exc))

    def display_tracks(self):
        if self.route_csv is None:
            return [dict(label='固件定位轨迹' if self.nav_mode else '实时路线',cross=[p[0] for p in self.route_live],along=[p[1] for p in self.route_live])] if self.route_live else []
        return self.route_tracks

    def refresh_route(self, force=False):
        if not self.route_dirty or (not force and time.monotonic()-self.route_last_draw<.1):
            return
        self.route_last_draw=time.monotonic()
        tracks=self.display_tracks()
        dpg.delete_item('route_y',children_only=True)
        for t in tracks:
            x,y=t['cross'],t['along']
            if not x:
                continue
            step=max(1,math.ceil(len(x)/10000))
            indices=list(range(0,len(x),step))
            if indices[-1]!=len(x)-1:
                indices.append(len(x)-1)
            dpg.add_line_series([x[i] for i in indices],[y[i] for i in indices],label=t['label'],parent='route_y')
            dpg.add_scatter_series([x[0]],[y[0]],label='起点',parent='route_y')
            end=dpg.add_scatter_series([x[-1]],[y[-1]],label='终点',parent='route_y')
            with dpg.theme() as theme:
                with dpg.theme_component(dpg.mvScatterSeries):
                    dpg.add_theme_style(dpg.mvPlotStyleVar_Marker,dpg.mvPlotMarker_Cross,category=dpg.mvThemeCat_Plots)
            dpg.bind_item_theme(end,theme)
            # Themes are reclaimed on next redraw with their series.
            if not hasattr(self,'route_themes'):
                self.route_themes=[]
            self.route_themes.append(theme)
        old=getattr(self,'route_old_themes',[])
        for theme in old:
            dpg.delete_item(theme)
        self.route_old_themes=getattr(self,'route_themes',[])
        self.route_themes=[]
        if dpg.get_value('route_fit'):
            dpg.fit_axis_data('route_x'); dpg.fit_axis_data('route_y')
        end=(tracks[-1]['cross'][-1],tracks[-1]['along'][-1]) if tracks else (0.,0.)
        dpg.set_value('route_status',f'{"CSV" if self.route_csv else "实时"} | 样本 {self.route_count} | 路程 {self.route_length:.1f} cm | 终点 X={end[0]:+.1f}, Y={end[1]:+.1f} cm')
        self.draw_navigation()
        self.route_dirty=False

    def export_route(self):
        try:
            tracks=self.display_tracks()
            if not tracks:
                raise ValueError('暂无路线数据')
            path=Path(dpg.get_value('route_output').strip())
            with path.open('w',newline='',encoding='utf-8-sig') as f:
                writer=csv.writer(f); writer.writerow(['track','sample','x_cm','y_cm'])
                for t in tracks:
                    writer.writerows((t['label'],i,x,y) for i,(x,y) in enumerate(zip(t['cross'],t['along'])))
            dpg.set_value('route_status',f'路线已导出：{path.resolve()}')
        except (OSError,ValueError) as exc:
            dpg.set_value('route_status',f'导出失败：{exc}')
