#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 Mcar 遥测 CSV 重建成俯视行走路线图（类似参考图第 3 张图）。

支持两类 CSV：
  1) 旧 GUI 导出的 CSV：nav_yaw_deg + ul/ur/dl/dr_cmps，没有时间戳；
  2) telemetry_logger.py 导出的 CSV：带 t_ms、nav_yaw_deg 和四轮 cm/s。

用法：
  python plot_mcar_path.py imu_data.csv
  python plot_mcar_path.py T3.csv T4.csv --labels T3 T4 --output compare.png
  python plot_mcar_path.py imu_data.csv --segment 4
  python plot_mcar_path.py imu_data.csv --segment all --output all_bouts.png

默认自动从单个 CSV 中选出一段最像“出去再回来”的闭合运动段，避免把停车间隔很长的
多次试跑累积成一团。旧 GUI CSV 默认按固件 100 Hz 控制周期（dt=10 ms）积分；如果
该 CSV 的实际采样间隔不同，请用 --dt-ms 指定。若 CSV 含 t_ms/t_s 时间列，则优先使用时间戳。

路径使用四轮实测速度反算车体前进/左移速度，再按固件的世界系公式用 nav_yaw_deg 积分。
每个 CSV 都各自把起点设为 (0, 0)，并把首个航向作为 0°，方便叠加比较多次测试。
与固件坐标一致：初始航向为 0° 时，前进映射到世界 +X，左移映射到世界 +Y；因此纯横移测试会沿图中 Y 方向形成主轨迹。
"""

import argparse
import csv
import math
import sys
from pathlib import Path

WHEELS = ("ul", "ur", "dl", "dr")
# 对应固件 Motor.h / app_navigation.c 中的标定默认值
DEFAULT_SCALE_X = 0.97
DEFAULT_SCALE_Y = 1.04
DEFAULT_LATERAL_FACTOR = 0.901589


def as_float(value):
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        number = float(text)
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def load_csv(path):
    """读入有效遥测行，适配旧 GUI CSV 和 telemetry_logger CSV。"""
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError("CSV 没有表头")
        # 统一大小写/空格，兼容 UTF-8 BOM。
        names = {str(name).strip().lstrip("\ufeff").lower(): name for name in reader.fieldnames if name}

        required = [w + "_cmps" for w in WHEELS]
        missing = [name for name in required if name.lower() not in names]
        if missing:
            raise ValueError("缺少四轮速度列：%s" % ", ".join(missing))

        yaw_key = "nav_yaw_deg" if "nav_yaw_deg" in names else ("yaw_deg" if "yaw_deg" in names else None)
        if yaw_key is None:
            raise ValueError("找不到 nav_yaw_deg（或备用 yaw_deg）航向列")

        time_key = None
        time_factor = 1.0
        if "t_ms" in names:
            time_key, time_factor = "t_ms", 0.001
        elif "time_ms" in names:
            time_key, time_factor = "time_ms", 0.001
        elif "t_s" in names:
            time_key, time_factor = "t_s", 1.0
        elif "time_s" in names:
            time_key, time_factor = "time_s", 1.0

        samples = []
        for raw in reader:
            row = {str(k).strip().lstrip("\ufeff").lower(): v for k, v in raw.items() if k}
            yaw = as_float(row.get(yaw_key))
            wheels = [as_float(row.get((w + "_cmps").lower())) for w in WHEELS]
            # 旧 GUI 文件前段可能只录了 IMU 姿态；跳过不完整控制行。
            if yaw is None or any(v is None for v in wheels):
                continue
            time_s = as_float(row.get(time_key)) * time_factor if time_key and as_float(row.get(time_key)) is not None else None
            samples.append({"time_s": time_s, "yaw_deg": yaw, "wheels": wheels})

    if len(samples) < 2:
        raise ValueError("有效数据不足：至少需要两行航向和四轮 cm/s 数据")
    return samples, yaw_key, time_key


def unwrap_degrees(values):
    """展开 ±180° 跳变，避免过零时把航向插值成错误的大角度。"""
    out = [values[0]]
    last_raw = values[0]
    for value in values[1:]:
        delta = (value - last_raw + 180.0) % 360.0 - 180.0
        out.append(out[-1] + delta)
        last_raw = value
    return out


def body_velocity(wheels):
    """按 Mecanum 轮序 UL, UR, DL, DR 反算车体前进/左移速度，单位 cm/s。"""
    ul, ur, dl, dr = wheels
    forward = 0.25 * (ul + ur + dl + dr)
    left = 0.25 * (-ul + ur + dl - dr)
    return forward, left


def reconstruct(samples, args):
    """积分得到横向/纵向路径，并返回统计数据。"""
    yaws = unwrap_degrees([s["yaw_deg"] for s in samples])
    yaw0 = yaws[0]
    body = [body_velocity(s["wheels"]) for s in samples]

    cross = [0.0]  # 固件世界系 X；初始航向为 0° 时前进为正
    along = [0.0]  # 固件世界系 Y；初始航向为 0° 时左移为正
    path_length = 0.0
    skipped_gaps = 0
    dt_fallbacks = 0
    default_dt = args.dt_ms / 1000.0
    max_gap = args.max_gap_ms / 1000.0

    for i in range(1, len(samples)):
        t0, t1 = samples[i - 1]["time_s"], samples[i]["time_s"]
        if t0 is not None and t1 is not None and t1 > t0:
            dt = t1 - t0
        else:
            dt = default_dt
            dt_fallbacks += 1

        # 大时间戳间隙通常代表漏包/暂停；不把末端速度错误地外推整段时间。
        if dt > max_gap + 1e-9:  # 加 epsilon，避免 100 ms 的浮点表示误判为超限
            cross.append(cross[-1])
            along.append(along[-1])
            skipped_gaps += 1
            continue
        if dt <= 0.0:
            cross.append(cross[-1])
            along.append(along[-1])
            continue

        fwd = 0.5 * (body[i - 1][0] + body[i][0])
        lat = 0.5 * (body[i - 1][1] + body[i][1])

        # 复现固件定位里程计标定：前进 scale_x；横移乘旧横移系数与 scale_y。
        dx = fwd * dt * args.scale_x
        dy = lat * dt * args.lateral_factor * args.scale_y

        # 可选斜向横移补偿（默认 1.0，即关闭）。
        if args.diag_comp != 1.0:
            adx, ady = abs(dx), abs(dy)
            mix = min(adx, ady) / max(adx, ady) if max(adx, ady) > 1e-9 else 0.0
            dy *= 1.0 + (args.diag_comp - 1.0) * mix

        # 固件的 lateral-to-forward coupling，默认 0。
        dx += args.lateral_coupling * dy

        # 与 navigation_fusion.c 一致：X=forward*cos - left*sin，Y=forward*sin + left*cos。
        theta = math.radians(0.5 * (yaws[i - 1] + yaws[i]) - yaw0)
        dx_world = dx * math.cos(theta) - dy * math.sin(theta)
        dy_world = dx * math.sin(theta) + dy * math.cos(theta)
        cross.append(cross[-1] + dx_world)
        along.append(along[-1] + dy_world)
        path_length += math.hypot(dx_world, dy_world)

    net_cross, net_along = cross[-1], along[-1]
    summary = {
        "samples": len(samples),
        "path_cm": path_length,
        "net_cross_cm": net_cross,
        "net_along_cm": net_along,
        "max_cross_cm": max(cross, key=abs),
        "skipped_gaps": skipped_gaps,
        "dt_fallbacks": dt_fallbacks,
    }
    return cross, along, summary


def find_motion_bouts(samples, args):
    """按四轮实测速度切分运动段；默认静止/断档 0.5 s 后开始下一段。"""
    bouts = []
    start = None
    last_active = None
    gap_s = args.split_gap_s
    active_th = args.move_threshold
    default_dt = args.dt_ms / 1000.0

    for i, sample in enumerate(samples):
        moving = max(abs(v) for v in sample["wheels"]) >= active_th
        if moving:
            if start is None:
                start = i
            last_active = i
            continue
        if start is None:
            continue

        t_last = samples[last_active]["time_s"]
        t_now = sample["time_s"]
        if t_last is not None and t_now is not None and t_now >= t_last:
            inactive_s = t_now - t_last
        else:
            inactive_s = (i - last_active) * default_dt
        if inactive_s >= gap_s:
            bouts.append((start, last_active))
            start = None
            last_active = None

    if start is not None and last_active is not None:
        bouts.append((start, last_active))
    return bouts


def choose_bouts(samples, bouts, args, path):
    """根据 --segment 选择手动编号、全部段，或最像闭合往返的一段。"""
    if not bouts:
        print("提示：%s 没检出明显运动段，将使用整份有效数据。" % path)
        return [(0, len(samples) - 1, None)]

    spec = str(args.segment).strip().lower()
    if spec == "all":
        return [(a, b, i + 1) for i, (a, b) in enumerate(bouts)]

    if spec != "auto":
        try:
            number = int(spec)
        except ValueError:
            raise ValueError("--segment 只能是 auto、all 或正整数")
        if number < 1 or number > len(bouts):
            raise ValueError("%s 只检出 %d 段；--segment 编号应为 1..%d" % (path, len(bouts), len(bouts)))
        a, b = bouts[number - 1]
        return [(a, b, number)]

    scored = []
    for number, (a, b) in enumerate(bouts, 1):
        x, y, stats = reconstruct(samples[a:b + 1], args)
        net = math.hypot(stats["net_cross_cm"], stats["net_along_cm"])
        excursion = max(math.hypot(px, py) for px, py in zip(x, y))
        closure = net / stats["path_cm"] if stats["path_cm"] > 1e-6 else 1.0
        scored.append((number, a, b, stats["path_cm"], closure, excursion))

    # 优先选路径够长、终点回到起点附近的闭环；没有闭环时退回到最长运动段。
    closed = [s for s in scored if s[3] >= 30.0 and s[5] >= 10.0 and s[4] <= 0.15]
    pool = closed or scored
    best = max(pool, key=lambda s: (s[3], -s[4]))
    number, a, b, length, closure, excursion = best
    if closed:
        others = [str(s[0]) for s in closed if s[0] != number]
        suffix = "；其他往返候选段：%s" % ", ".join(others) if others else ""
        print("自动选择 %s 第 %d/%d 段作为单次往返（路径≈%.1f cm，闭合误差≈%.1f cm）%s。" % (
            path, number, len(bouts), length, closure * length, suffix))
    else:
        print("%s 未找到明显闭合往返，自动选择路径最长的第 %d/%d 段（≈%.1f cm）。" % (
            path, number, len(bouts), length))
    return [(a, b, number)]


def default_output(paths):
    if len(paths) == 1:
        return str(Path(paths[0]).with_name(Path(paths[0]).stem + "_path.png"))
    return "mcar_path_compare.png"


def make_plot(tracks, output, dpi):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("需要安装 matplotlib：python -m pip install matplotlib") from exc

    fig, ax = plt.subplots(figsize=(7.5, 7.0))
    for track in tracks:
        line, = ax.plot(track["cross"], track["along"], linewidth=2.0, label=track["label"])
        color = line.get_color()
        ax.scatter(track["cross"][0], track["along"][0], marker="o", s=48,
                   color=color, edgecolors="white", linewidths=0.8, zorder=4)
        ax.scatter(track["cross"][-1], track["along"][-1], marker="X", s=65,
                   color=color, edgecolors="white", linewidths=0.8, zorder=4)

    ax.axhline(0, color="0.55", linewidth=0.8, alpha=0.45)
    ax.axvline(0, color="0.55", linewidth=0.8, alpha=0.45)
    ax.grid(True, alpha=0.28)
    ax.set_aspect("equal", adjustable="datalim")
    ax.margins(0.08)
    ax.set_xlabel("cross-track x (cm)")
    ax.set_ylabel("along-track y (cm)")
    ax.set_title("World-frame path from wheel odometry + nav yaw")
    ax.text(0.02, 0.98, "○ start     × end", transform=ax.transAxes,
            ha="left", va="top", fontsize=9,
            bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "alpha": 0.8, "edgecolor": "0.8"})
    if len(tracks) > 1:
        ax.legend(loc="best")
    elif tracks:
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="从四轮实测速度 + nav_yaw 重建 Mcar 的俯视行走路线。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="\n".join([
            "示例：",
            "  python plot_mcar_path.py imu_data.csv",
            "  python plot_mcar_path.py T3.csv T4.csv --labels T3 T4 --output compare.png",
            "  python plot_mcar_path.py old_gui.csv --dt-ms 13.1 --output route.png",
        ]),
    )
    parser.add_argument("csv", nargs="+", help="一个或多个 CSV；多文件会叠加比较")
    parser.add_argument("--labels", nargs="+", help="图例名称，数量须与 CSV 数量一致")
    parser.add_argument("--output", "-o", help="PNG 输出路径；默认 <CSV名>_path.png")
    parser.add_argument("--dt-ms", type=float, default=10.0,
                        help="无时间戳 CSV 的假定采样间隔，毫秒（默认 10；固件控制周期 100 Hz）")
    parser.add_argument("--max-gap-ms", type=float, default=100.0,
                        help="时间戳相邻间隔超过此值时跳过该步，避免漏包造成虚假长位移（默认 100）")
    parser.add_argument("--scale-x", type=float, default=DEFAULT_SCALE_X,
                        help="前进里程比例（默认固件值 0.97）")
    parser.add_argument("--scale-y", type=float, default=DEFAULT_SCALE_Y,
                        help="横移菜单比例（默认固件值 1.04）")
    parser.add_argument("--lateral-factor", type=float, default=DEFAULT_LATERAL_FACTOR,
                        help="Motor.h 横移修正系数（默认 0.901589）")
    parser.add_argument("--diag-comp", type=float, default=1.0,
                        help="斜向横移补偿系数；默认 1.0=关闭")
    parser.add_argument("--lateral-coupling", type=float, default=0.0,
                        help="横移引起的前后耦合；默认 0")
    parser.add_argument("--segment", default="auto",
                        help="选择运动段：auto=自动选最像往返的一段（默认），正整数=指定段号，all=绘制全部段")
    parser.add_argument("--move-threshold", type=float, default=3.0,
                        help="轮速超过此值视为运动，cm/s（默认 3）")
    parser.add_argument("--split-gap-s", type=float, default=0.5,
                        help="连续静止多久后切成新运动段，秒（默认 0.5）")
    parser.add_argument("--dpi", type=int, default=180, help="图片分辨率（默认 180 dpi）")
    args = parser.parse_args(argv)
    if args.labels and len(args.labels) != len(args.csv):
        parser.error("--labels 数量必须与 CSV 文件数量一致")
    if args.dt_ms <= 0 or args.max_gap_ms <= 0:
        parser.error("--dt-ms 和 --max-gap-ms 必须大于 0")
    if args.move_threshold < 0 or args.split_gap_s <= 0:
        parser.error("--move-threshold 不能小于 0，--split-gap-s 必须大于 0")
    if args.dpi <= 0:
        parser.error("--dpi 必须大于 0")
    return args


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    labels = args.labels or [Path(p).stem for p in args.csv]
    tracks = []

    for path, label in zip(args.csv, labels):
        try:
            samples, yaw_key, time_key = load_csv(path)
            bouts = find_motion_bouts(samples, args)
            selections = choose_bouts(samples, bouts, args, path)
        except (OSError, csv.Error, ValueError) as exc:
            print("读取 %s 失败：%s" % (path, exc), file=sys.stderr)
            return 2
        if yaw_key == "yaw_deg":
            print("警告：%s 没有 nav_yaw_deg，改用 yaw_deg；若 IMU 安装角/方向不同，路线朝向可能不准。" % path)
        if time_key is None:
            print("提示：%s 无时间戳，按 dt=%.3f ms 积分。" % (path, args.dt_ms))

        for a, b, segment_number in selections:
            chosen = samples[a:b + 1]
            cross, along, summary = reconstruct(chosen, args)
            if time_key is not None and summary["dt_fallbacks"]:
                print("提示：%s 有 %d 个间隔缺少/异常时间戳，使用默认 dt=%.3f ms。" % (
                    path, summary["dt_fallbacks"], args.dt_ms))
            if summary["skipped_gaps"]:
                print("警告：%s 跳过了 %d 个超过 %.1f ms 的时间间隔；可能有漏包。" % (
                    path, summary["skipped_gaps"], args.max_gap_ms))

            track_label = label
            if args.segment == "all" and segment_number is not None:
                track_label = "%s #%d" % (label, segment_number)
            tracks.append({"label": track_label, "cross": cross, "along": along})
            print("%-24s 段=%s  样本=%d  路径长≈%.1f cm  终点=(x %+.1f, y %+.1f) cm  最大|x|=%+.1f cm" % (
                track_label, str(segment_number or "全段"), summary["samples"], summary["path_cm"],
                summary["net_cross_cm"], summary["net_along_cm"], summary["max_cross_cm"]))

    output = args.output or default_output(args.csv)
    try:
        make_plot(tracks, output, args.dpi)
    except (OSError, RuntimeError) as exc:
        print("生成图片失败：%s" % exc, file=sys.stderr)
        return 2
    print("路线图已保存：%s" % output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
