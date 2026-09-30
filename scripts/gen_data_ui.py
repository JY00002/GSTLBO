#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""地面站数据生成器（UI 调用入口 + 目标/禁飞区采样工具函数，一体化单文件）。

功能：
  1. 接收地面站 UI（saveTaskPoints 格式）的参数文件，生成地面站可直接加载的
     .plan 场景文件：目标点(polygon)、禁飞区(radars)、模拟新目标(sim_new_targets)、
     可选模拟失效事件(sim_drone_failures)、可选 VRPRD 释放时间(trigger_times)。
  2. 内置 WGS84↔ENU 坐标转换、矩形区域目标采样、网格抖动禁飞区采样与 .plan 写出工具。

用法: python gen_data_ui.py <params_file>
  params_file: saveTaskPoints 格式 JSON，polygon 的元素编码参数:
    [0]: ref_lat, [1]: ref_lon, [2]: ref_alt,
    [3]: num_drones, [4]: num_targets, [5]: x_range, [6]: y_range,
    [7]: samples, [8]: seed, [9]: n_radars, [10]: max_range_minutes,
    [11]: radius_min, [12]: radius_max,       # NFZ 半径范围（米）
    [13]: num_dynamic, [14]: dyn_t_min, [15]: dyn_t_max,
    [16]: gen_mode,                           # 0=运行时注入(sim_new_targets), 1=VRPRD(释放时间)
    [17]: num_fail, [18]: fail_t_min, [19]: fail_t_max,   # 模拟无人机失效（时间单位: 秒）
    [20:]: 每架无人机 5 元组 (type_id, speed, lat, lon, alt)，type_id ∈ {0:recon,1:combat,2:eval}
  gen_mode=0(默认): 模拟新目标(机载识别占位)写入 mission.sim_new_targets，不入 polygon，
  运行中出现后触发重规划。
  gen_mode=1(VRPRD): 动态目标写入 polygon 末尾 + mission.trigger_times(长度 N, 静态在前填0)+
  mission.num_dynamic，动态目标到释放时间才可调度；规划请求带 num_dynamic/dynamic_trigger_times。

本文件由 gen_data_ui.py 与 generate_swarmalloc_targets.py 合并而来（单文件、无本地依赖），
并修复了两处与地面站 UI 实际参数布局不一致的问题：
  · 无人机 5 元组自第 20 个值开始解析（UI 在 [17..19] 写入失效三元组；旧版从 17 起解析
    会把失效参数误当无人机字段，已在注释中标明）；
  · 无人机解析上限由 8 提高到 16（与 UI 的 16 个无人机输入框一致）。
"""
import json
import math
import os
import sys
from typing import List, Tuple

import numpy as np


# ==============================================================================
# WGS84 ↔ ENU 坐标转换（小区域平坦地球近似，精度 < 0.1% @ < 10km）
# ==============================================================================

# WGS84 椭球参数
_A = 6378137.0                # 半长轴 (m)
_E2 = 6.69437999014e-3        # 第一偏心率平方
_DEG_TO_RAD = math.pi / 180.0


def _radius_of_curvature(lat_rad: float) -> Tuple[float, float]:
    """计算卯酉圈曲率半径 Rn 和子午圈曲率半径 Rm。"""
    sin_lat = math.sin(lat_rad)
    w = math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
    rn = _A / w                           # 卯酉圈 (east-west)
    rm = _A * (1.0 - _E2) / (w * w * w)   # 子午圈 (north-south)
    return rn, rm


def wgs84_to_enu(lat: float, lon: float, alt: float,
                 lat_ref: float, lon_ref: float, alt_ref: float
                 ) -> Tuple[float, float, float]:
    """WGS84 经纬高 → ENU (东-北-天, 米)。"""
    lat_r = lat_ref * _DEG_TO_RAD
    rn, rm = _radius_of_curvature(lat_r)

    dlat = (lat - lat_ref) * _DEG_TO_RAD
    dlon = (lon - lon_ref) * _DEG_TO_RAD

    north = rm * dlat
    east = rn * math.cos(lat_r) * dlon
    up = alt - alt_ref

    return east, north, up


def enu_to_wgs84(east: float, north: float, up: float,
                 lat_ref: float, lon_ref: float, alt_ref: float
                 ) -> Tuple[float, float, float]:
    """ENU (东-北-天, 米) → WGS84 经纬高。"""
    lat_r = lat_ref * _DEG_TO_RAD
    rn, rm = _radius_of_curvature(lat_r)

    dlat = north / rm
    dlon = east / (rn * math.cos(lat_r))

    lat = lat_ref + dlat / _DEG_TO_RAD
    lon = lon_ref + dlon / _DEG_TO_RAD
    alt = alt_ref + up

    return lat, lon, alt


# ==============================================================================
# 目标点生成（矩形区域）
# ==============================================================================

INT_TO_TYPE = {0: "recon", 1: "combat", 2: "eval"}


def generate_targets_rect_enu(num_targets: int,
                              x_range: float, y_range: float,
                              nfz_centers: List[Tuple[float, float]] = None,
                              nfz_radii: List[float] = None,
                              min_separation: float = 30.0,
                              rng: np.random.RandomState = None) -> np.ndarray:
    """在矩形区域 [0, x_range] × [0, y_range] 内均匀生成目标点（ENU 米）。

    目标点以 ENU 坐标原点 (0,0) 为矩形左下角，向东 x_range 米、向北 y_range 米。
    如果提供了禁飞区，则避开禁飞区圆（含 10m 安全边距）。

    Args:
        num_targets: 要生成的目标点数量
        x_range: 矩形东向范围 (米)
        y_range: 矩形北向范围 (米)
        nfz_centers: 禁飞区圆心列表 [(east, north), ...]，可选
        nfz_radii: 禁飞区半径列表 [r, ...]，可选
        min_separation: 目标点之间的最小间距 (米)
        rng: 随机数生成器

    Returns:
        targets_enu: shape [num_targets, 2], (east, north) ENU 坐标
    """
    if rng is None:
        rng = np.random.RandomState()

    if nfz_centers is None:
        nfz_centers = []
    if nfz_radii is None:
        nfz_radii = []

    targets = []
    # 30m 互斥 + NFZ 避让下随机采样可能难以放满 num_targets（原 max_attempts=num_targets*50
    # 在小区域/多 NFZ 时经常只返回 9/10 个）。修复：增大尝试次数，并在不足时逐级放宽
    # 最小间距（30m → 18m → 10m），保证最终生成足量目标（间距需求优先，区域放不下才降档）。
    for _sep in (min_separation, max(min_separation * 0.6, 10.0), 10.0):
        if len(targets) >= num_targets:
            break
        targets.clear()
        attempts = 0
        max_attempts = max(num_targets * 200, 2000)
        while len(targets) < num_targets and attempts < max_attempts:
            # 矩形区域内均匀采样
            ex = rng.uniform(0.0, x_range)
            ey = rng.uniform(0.0, y_range)

            too_close = False

            # 检查与已有目标点的最小间距
            for t in targets:
                if math.hypot(ex - t[0], ey - t[1]) < _sep:
                    too_close = True
                    break

            # 检查是否落入禁飞区（含 10m 安全边距）
            if not too_close:
                for (cx, cy), r in zip(nfz_centers, nfz_radii):
                    if math.hypot(ex - cx, ey - cy) <= r + 10.0:
                        too_close = True
                        break

            if not too_close:
                targets.append([ex, ey])

            attempts += 1

    if len(targets) < num_targets:
        print(f"警告: 仅生成了 {len(targets)}/{num_targets} 个目标点 "
              f"(矩形={x_range:.0f}×{y_range:.0f}m, 最小间距={min_separation}m, "
              f"NFZ数={len(nfz_centers)})")

    return np.array(targets)


# ==============================================================================
# 禁飞区（敌方雷达）生成 —— 复刻 parco HDSVRPGenerator 的网格抖动算法
# (parco/envs/hdsvrp/generator.py: _generate 中 NFZ 部分)
# ==============================================================================

# 与 parco 默认一致：NFZ 是圆，半径 40~60m，数量可由调用方指定（parco 默认 5~10）
NFZ_RADIUS_MIN = 40.0
NFZ_RADIUS_MAX = 60.0


def generate_nfz_rect_enu(x_range: float, y_range: float,
                          targets_enu: List[Tuple[float, float]],
                          num_nfz: int,
                          rng: np.random.RandomState = None,
                          radius_min: float = NFZ_RADIUS_MIN,
                          radius_max: float = NFZ_RADIUS_MAX
                          ) -> Tuple[List[Tuple[float, float]], List[float]]:
    """在矩形区域 [0, x_range] × [0, y_range] 内生成圆形禁飞区（敌方雷达）。

    使用网格抖动算法，把矩形区域切成 ceil(sqrt(n)) 的网格，
    每格中心 + ±20% 格宽的随机偏移，保证 NFZ 铺开不扎堆。
    避开所有已生成的目标点。

    Args:
        x_range: 矩形东向范围 (米)
        y_range: 矩形北向范围 (米)
        targets_enu: 已生成的目标点 ENU 坐标 [(east, north), ...]
        num_nfz: NFZ 数量
        rng: 随机数生成器
        radius_min: NFZ 半径下界（米），默认 40
        radius_max: NFZ 半径上界（米），默认 60

    Returns:
        nfz_centers: [(east, north), ...] 每个 NFZ 圆心（ENU 米）
        nfz_radii:  [r, ...] 每个 NFZ 半径（米）
    """
    if num_nfz <= 0:
        return [], []

    if rng is None:
        rng = np.random.RandomState()
    r_lo = max(0.0, float(radius_min))
    r_hi = max(0.0, float(radius_max))
    if r_lo > r_hi:
        r_lo, r_hi = r_hi, r_lo

    # 网格抖动：ceil(sqrt(n)) 网格，覆盖整个矩形区域
    grid_size = int(math.ceil(math.sqrt(num_nfz)))
    margin_ratio = 0.10  # 内边距 10%，避免 NFZ 贴边
    mx = x_range * margin_ratio
    my = y_range * margin_ratio
    cell_w = (x_range - 2 * mx) / grid_size if grid_size > 0 else x_range
    cell_h = (y_range - 2 * my) / grid_size if grid_size > 0 else y_range
    jitter_e = cell_w * 0.2  # 20% 抖动
    jitter_n = cell_h * 0.2

    nfz_centers = []
    nfz_radii = []

    for idx in range(num_nfz):
        row = idx // grid_size
        col = idx % grid_size
        base_e = mx + (col + 0.5) * cell_w
        base_n = my + (row + 0.5) * cell_h
        off_e = rng.uniform(-jitter_e, jitter_e)
        off_n = rng.uniform(-jitter_n, jitter_n)
        cx = max(0.0, min(x_range, base_e + off_e))
        cy = max(0.0, min(y_range, base_n + off_n))
        r = float(rng.uniform(r_lo, r_hi))

        # 约束 1：避开所有目标点（含 10m 安全边距）
        bad = False
        for t_e, t_n in targets_enu:
            if math.hypot(cx - t_e, cy - t_n) <= r + 10.0:
                bad = True
                break

        # 约束 2：与已有 NFZ 不过度重叠（圆心距 > 两半径和的 60%）
        if not bad:
            for (ex, ey), er in zip(nfz_centers, nfz_radii):
                if math.hypot(cx - ex, cy - ey) < (r + er) * 0.6:
                    bad = True
                    break

        if bad:
            continue  # 跳过该网格

        nfz_centers.append((cx, cy))
        nfz_radii.append(r)

    if len(nfz_centers) < num_nfz:
        print(f"警告: 仅生成了 {len(nfz_centers)}/{num_nfz} 个雷达 "
              f"(矩形={x_range:.0f}×{y_range:.0f}m, 避开目标/重叠约束)")

    return nfz_centers, nfz_radii


# ==============================================================================
# .plan 文件输出
# ==============================================================================

def write_plan_file(targets_wgs84: List[Tuple[float, float, float]],
                    output_path: str,
                    radars: List[dict] = None):
    """写入地面站兼容的 .plan JSON 文件。

    Args:
        targets_wgs84: 目标点 [(lat, lon, alt), ...]
        output_path: 输出路径
        radars: 敌方雷达列表 [{"lat":.., "lon":.., "radius":米}, ...]，可选。
                地面站加载 .plan 时读取并显示为"敌方雷达区域"圆，
                同时在规划时转成 swarmalloc 的 nfz_centers/nfz_radii。
    """
    item = {
        "polygon": [
            [lat, lon, alt] for lat, lon, alt in targets_wgs84
        ]
    }
    if radars:
        item["radars"] = radars
    plan = {
        "mission": {
            "items": [item]
        }
    }
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(plan, f, indent=2, ensure_ascii=False)
    print(f"✓ 目标点文件已写入: {output_path}")
    print(f"  目标数: {len(targets_wgs84)}")
    for i, (lat, lon, alt) in enumerate(targets_wgs84[:5]):
        print(f"  [{i}] lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}m")
    if len(targets_wgs84) > 5:
        print(f"  ... 共 {len(targets_wgs84)} 个目标点")
    if radars:
        print(f"  雷达数: {len(radars)}")
        for i, r in enumerate(radars[:5]):
            print(f"  [雷达{i}] lat={r['lat']:.6f}, lon={r['lon']:.6f}, radius={r['radius']:.1f}m")


# ==============================================================================
# 地面站 UI 调用入口
# ==============================================================================

def main():
    if len(sys.argv) < 2:
        print("用法: python gen_data_ui.py <params_file>")
        sys.exit(1)

    # 读取 saveTaskPoints 格式的参数文件
    with open(sys.argv[1]) as f:
        data = json.load(f)

    polygon = data["mission"]["items"][0]["polygon"]
    # saveTaskPoints 把所有参数拍平成一个 polygon 数组，每 3 个值一组：
    #   polygon[0] = [ref_lat, ref_lon, ref_alt]               参考点
    #   polygon[1] = [n_drones, n_targets, x_range, y_range]   规模参数
    #   polygon[2] = [samples, seed, n_radars, maxRange, radiusMin, radiusMax]  生成参数
    #   polygon[3] = [num_dynamic, dyn_t_min, dyn_t_max]       动态参数
    #   polygon[4:] 展开为扁平列表，后续字段按扁平索引解析
    flat_pts = []  # 扁平标量列表
    for entry in polygon:
        if isinstance(entry, list):
            flat_pts.extend(entry)
        else:
            flat_pts.append(entry)

    def _get(idx, default=0):
        return flat_pts[idx] if idx < len(flat_pts) else default

    ref_lat  = float(_get(0))
    ref_lon  = float(_get(1))
    ref_alt  = float(_get(2))
    n_drones  = int(_get(3, 3))
    n_targets = int(_get(4, 10))
    x_range   = int(_get(5, 1000))
    y_range   = int(_get(6, 1000))
    n_samples = int(_get(7, 1))
    seed      = int(_get(8, 42))
    # 雷达(NFZ)数量：第 9 个值，未提供则默认 0（不生成雷达）
    n_radars  = int(_get(9, 0))
    # 最大航程（分钟），默认 15 分钟 = 900 秒；0 表示无限（向后兼容旧 paramsPoints 无此字段）
    max_range_minutes = int(_get(10, 15))
    # NFZ 半径范围（米）：第 11/12 个值，默认 [40, 60]
    radius_min = float(_get(11, 40.0))
    radius_max = float(_get(12, 60.0))
    # 模拟新目标数：第 13 个值，默认 0（不生成模拟新目标）
    num_dynamic = int(_get(13, 0))
    num_dynamic = max(0, min(num_dynamic, n_targets))
    # 模拟新目标出现时间范围（秒）：第 14/15 个值，默认 [60, 180]
    dyn_t_min = float(_get(14, 60.0))
    dyn_t_max = float(_get(15, 180.0))
    if dyn_t_max <= dyn_t_min:
        dyn_t_max = dyn_t_min + 1.0
    # 生成模式：第 16 个值。0=运行时注入(sim_new_targets)，1=VRPRD(释放时间)
    # （防御性钳制：旧参数文件 index 16 是首机 type，type=1(combat) 会被误判为 VRPRD；
    #   实际 UI 每次点击都会重写参数文件，此边界仅影响手动跑脚本的旧文件。）
    gen_mode = int(_get(16, 0))
    gen_mode = 1 if gen_mode == 1 else 0
    # 无人机失效事件：第 17/18/19 个值 = [失效机数, 失效时间min, 失效时间max]。默认 0(不生成)。
    num_fail    = int(_get(17, 0))
    fail_t_min  = float(_get(18, 60.0))
    fail_t_max  = float(_get(19, 180.0))
    num_fail = max(0, min(num_fail, n_drones))
    if fail_t_max <= fail_t_min:
        fail_t_max = fail_t_min + 1.0

    reference = (ref_lat, ref_lon, ref_alt)
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_data", "ui_generated")
    os.makedirs(out_dir, exist_ok=True)

    rng = np.random.RandomState(seed)

    # 解析无人机配置：扁平列表从第 20 个值开始，每 5 个值一组 (type_id, speed, lat, lon, alt)。
    # 索引布局（与地面站 UI 的 paramsPoints 一致）：
    #   [0..2] ref / [3..6] 规模 / [7..12] 生成参数 / [13..15] 动态 / [16] gen_mode
    #   / [17..19] 失效三元组 / [20..] 无人机 5 元组
    # 注意：旧版脚本曾自第 17 个值起解析无人机（把失效三元组误当无人机字段），此处已修正。
    drones_config = []
    drone_flat_start = 20
    for i in range(min(n_drones, 16)):
        off = drone_flat_start + i * 5
        if off + 4 >= len(flat_pts):
            break
        t = int(flat_pts[off])
        spd = float(flat_pts[off + 1])
        lat = float(flat_pts[off + 2])
        lon = float(flat_pts[off + 3])
        alt = float(flat_pts[off + 4])
        max_range_s = float(max_range_minutes * 60) if max_range_minutes > 0 else 1e9
        drones_config.append({"type": INT_TO_TYPE.get(t, "recon"), "speed": spd,
                              "lat": lat, "lon": lon, "alt": alt,
                              "max_range_s": max_range_s})

    if not drones_config:
        # 兜底默认无人机配置：depot 沿 Y=0 水平等距排列 (spacing=150m，与训练一致)
        # 类型按顺序循环 (recon,combat,eval)，ENU 坐标均匀分布在 [0, x_range] 水平线上
        ALT_CYCLE = [30.0, 40.0, 45.0]
        TYPE_CYCLE = ["recon", "combat", "eval"]
        SPEED = 15.0
        depot_spacing = 150.0
        total_width = (n_drones - 1) * depot_spacing
        center_x = x_range / 2.0
        start_x = center_x - total_width / 2.0
        drones_config = []
        for i in range(n_drones):
            enu_x = start_x + i * depot_spacing
            enu_y = 0.0
            lat, lon, _ = enu_to_wgs84(enu_x, enu_y, 0.0, ref_lat, ref_lon, ref_alt)
            drones_config.append({
                "type": TYPE_CYCLE[i % 3],
                "speed": SPEED,
                "lat": lat,
                "lon": lon,
                "alt": ALT_CYCLE[i % 3],
                "max_range_s": float(max_range_minutes * 60) if max_range_minutes > 0 else 1e9,
            })

    for si in range(n_samples):
        srng = np.random.RandomState(seed * 1000 + si)
        drones_enu = []
        for d in drones_config:
            e, n, _ = wgs84_to_enu(d["lat"], d["lon"], d["alt"], *reference)
            drones_enu.append((e, n))

        # 先占位生成目标点（用于NFZ避让），正式目标点后面会重新生成
        dummy_targets = [(float(x_range * srng.uniform(0.3, 0.7)),
                          float(y_range * srng.uniform(0.3, 0.7)))
                         for _ in range(min(n_targets, 5))]

        # 先生成敌方雷达（NFZ）：矩形区域内网格抖动，避开占位目标点
        nfz_centers_enu, nfz_radii = generate_nfz_rect_enu(
            x_range, y_range, dummy_targets, n_radars, srng,
            radius_min=radius_min, radius_max=radius_max)

        # 生成全部目标点（避开 NFZ）：
        #   sim 模式: n_targets 个已知目标入 polygon；
        #   VRPRD 模式: 一次性生成 N = n_targets + num_dynamic 个目标（保证两两 ≥30m 互斥），
        #               后 num_dynamic 个为动态目标（释放时间后才可调度）。
        total_targets = n_targets + num_dynamic if gen_mode == 1 else n_targets
        targets_enu = generate_targets_rect_enu(
            total_targets, x_range, y_range,
            nfz_centers=([(c[0], c[1]) for c in nfz_centers_enu] if nfz_centers_enu else None),
            nfz_radii=nfz_radii if nfz_radii else None,
            min_separation=30.0, rng=srng)

        targets_wgs84 = []
        for idx, (east, north) in enumerate(targets_enu):
            lat, lon, alt_wgs84 = enu_to_wgs84(east, north, 0.0, *reference)
            targets_wgs84.append((lat, lon, alt_wgs84))

        # VRPRD: 动态目标释放时间（长度 N，静态在前填 0、动态在后），与 plan_adapter 协议一致
        trigger_times = []
        if gen_mode == 1 and num_dynamic > 0:
            trigger_times = ([0.0] * n_targets
                             + list(srng.uniform(dyn_t_min, dyn_t_max, size=num_dynamic)))

        # 生成模拟新目标（机载识别占位）：不入 polygon，运行中到出现时刻触发重规划。
        # （VRPRD 模式不生成——动态目标已在 polygon + trigger_times 中）
        sim_new_targets = []
        if gen_mode == 0 and num_dynamic > 0:
            sim_enu = generate_targets_rect_enu(
                num_dynamic, x_range, y_range,
                nfz_centers=([(c[0], c[1]) for c in nfz_centers_enu] if nfz_centers_enu else None),
                nfz_radii=nfz_radii if nfz_radii else None,
                min_separation=30.0, rng=srng)
            sim_times = srng.uniform(dyn_t_min, dyn_t_max, size=len(sim_enu))
            for idx, (east, north) in enumerate(sim_enu):
                lat, lon, _ = enu_to_wgs84(east, north, 0.0, *reference)
                sim_new_targets.append({
                    "time": float(sim_times[idx]),
                    "lat": lat,
                    "lon": lon,
                    "alt": 40.0,
                })

        # 无人机失效事件：随机选 num_fail 架无人机, 在 [fail_t_min, fail_t_max] 内随机时刻失效
        # (测试用; 运行中到时刻由地面站标记失效 → 重规划重分配其未完成任务)
        sim_drone_failures = []
        if num_fail > 0:
            fail_ids = srng.choice(n_drones, size=num_fail, replace=False)
            fail_times = srng.uniform(fail_t_min, fail_t_max, size=num_fail)
            for fi, ft in zip(fail_ids, fail_times):
                sim_drone_failures.append({"time": float(ft), "drone_id": int(fi)})

        radars_wgs84 = []
        for (east, north), r in zip(nfz_centers_enu, nfz_radii):
            lat, lon, _ = enu_to_wgs84(east, north, 0.0, *reference)
            radars_wgs84.append({"lat": lat, "lon": lon, "radius": r})

        # 将参考点写入 .plan，确保加载侧使用一致的 WGS84↔ENU 转换
        plan_data = {
            "mission": {
                "reference": {"lat": ref_lat, "lon": ref_lon, "alt": ref_alt},
                "drones": [
                    {
                        "id": i,
                        "type": d["type"],
                        "speed": d["speed"],
                        "lat": d["lat"],
                        "lon": d["lon"],
                        "alt": d["alt"],
                        "max_range_s": d.get("max_range_s", 1e9),
                        "enu_x": float(drones_enu[i][0]),
                        "enu_y": float(drones_enu[i][1]),
                    }
                    for i, d in enumerate(drones_config)
                ],
                "items": [
                    {
                        "polygon": [
                            [lat, lon, alt] for lat, lon, alt in targets_wgs84
                        ],
                    }
                ],
            }
        }
        if radars_wgs84:
            plan_data["mission"]["items"][0]["radars"] = radars_wgs84
        if gen_mode == 1 and num_dynamic > 0:
            # VRPRD: 动态目标入 polygon 末尾 + 释放时间（长度 N，静态 0），无运行时注入
            plan_data["mission"]["trigger_times"] = trigger_times
            plan_data["mission"]["num_dynamic"] = num_dynamic
        elif sim_new_targets:
            plan_data["mission"]["sim_new_targets"] = sim_new_targets
        if sim_drone_failures:
            plan_data["mission"]["sim_drone_failures"] = sim_drone_failures

        range_tag = f"_r{max_range_minutes}" if max_range_minutes > 0 else ""
        # 动态目标文件名标签：包含数量 + 时间范围，便于按参数辨识样本
        sim_tag = ""
        if num_dynamic > 0:
            mode_tag = "vrprd" if gen_mode == 1 else "sim"
            sim_tag = f"_{mode_tag}{num_dynamic}_t{int(dyn_t_min)}-{int(dyn_t_max)}"
        plan_name = f"d{n_drones}_t{n_targets}{sim_tag}_x{x_range}_y{y_range}_n{n_radars}{range_tag}_s{seed}_{si:02d}.plan"
        plan_path = os.path.join(out_dir, plan_name)
        os.makedirs(os.path.dirname(plan_path) or ".", exist_ok=True)
        with open(plan_path, "w", encoding="utf-8") as f:
            json.dump(plan_data, f, indent=2, ensure_ascii=False)
        if gen_mode == 1 and num_dynamic > 0:
            dyn_info = (f", VRPRD动态={num_dynamic}(释放 {dyn_t_min:.0f}~{dyn_t_max:.0f}s, "
                        f"N={n_targets + num_dynamic})")
        elif num_dynamic > 0:
            dyn_info = f", 模拟新目标={num_dynamic}(出现 {dyn_t_min:.0f}~{dyn_t_max:.0f}s)"
        else:
            dyn_info = ""
        print(f"OK: 目标点文件已写入: {plan_path} (含参考点 + {len(drones_config)} 架无人机depot, 航程={max_range_minutes}min/{max_range_minutes*60}s{dyn_info})")
        if si == 0:
            main_plan_name = plan_name

    # 同时输出一个固定名 sample_00.plan 软链/复制，兼容旧加载逻辑；
    # 主输出用带参数的文件名
    main_plan = os.path.join(out_dir, main_plan_name)
    sample00 = os.path.join(out_dir, "sample_00.plan")
    try:
        import shutil
        shutil.copyfile(main_plan, sample00)
    except Exception:
        pass
    print(f"OK:{main_plan}")
    print(f"OK_COMPAT:{sample00}")


if __name__ == "__main__":
    main()
