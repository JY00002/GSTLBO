#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""数值实验基准数据生成器（一体化脚本：四个数据族共用一个入口）。

四个数据族（默认参数即论文口径，可复现仓库中已发布的数据）：

  randdepot  主基准     data/n{scale}_randdepot_real.npz
             在基础布局 *_real.npz 上把基地改为区域内均匀随机位置（随机基地）。
  nonuni     非均匀布局 data/n{scale}_nonuni_randdepot_real.npz
             目标点由高斯混合聚类生成（每实例 K∈[4,8] 个簇 + 约 10% 均匀噪声），基地随机。
  nfz        NFZ 族     data/n{scale}_randdepot_nfz_real.npz
             转弯半径设为 10 m，基地/目标重采样至距禁飞圆 ≥30 m，服务时间置 0
             （Dubins 下由绕飞项 πR/v 计时代替，避免重复计数）。
  dynamic    动态目标族 data/n{scale}_randdepot_dyn_real.npz
             末尾约 10% 目标标记为"执行中在线出现"（出现时间 100–300 s）。

用法：
  python gen_benchmark_data.py randdepot  [--scales n10_m3,n30_m6,...] [--data-dir PATH]
  python gen_benchmark_data.py nonuni     [--scales ...] [--data-dir PATH]
  python gen_benchmark_data.py nfz        [--scales ...] [--data-dir PATH]
                                          [--turning-radius 10] [--service-time 0]
                                          [--keep-service-time] [--no-randdepot]
  python gen_benchmark_data.py dynamic    [--scales ...] [--data-dir PATH]
                                          [--n-dyn-frac 0.1] [--seed 1002] [--t-min 100] [--t-max 300]
  python gen_benchmark_data.py all        [--data-dir PATH]      # 依次 randdepot → nonuni → nfz → dynamic

说明：
  · 输入为基础布局 data/n{scale}_real.npz（固定基地版）。基础布局未随仓库发布；
    缺失时对应规模会被跳过（SKIP），不影响其他规模。
  · 仅依赖 numpy，可本地直接运行。
  · 各族的随机种子内嵌为默认值，默认参数下输出与论文/仓库中的数据逐位一致。
"""
import argparse
import os
import sys

import numpy as np

# ---------------------------------------------------------------------------
# 公共设置
# ---------------------------------------------------------------------------
DEFAULT_SCALES = ["n10_m3", "n30_m6", "n60_m12", "n100_m16", "n150_m24", "n200_m32"]
DEFAULT_NFZ_SCALES = ["n30_m6", "n60_m12", "n100_m16", "n150_m24", "n200_m32"]
DEFAULT_DYN_SCALES = ["n60_m12", "n100_m16", "n150_m24", "n200_m32"]

REGION = 1000.0  # 与现有数据 locs 所在区域一致（1 km × 1 km）

# randdepot
SEED_RANDDEPOT = 42

# nonuni
SEED_NONUNI = 20260625   # 独立于 randdepot 的 SEED=42，避免共享 depot 随机流
NOISE_FRAC = 0.10        # 每实例噪声（均匀撒布）目标占比
K_MIN, K_MAX = 4, 8      # 每实例簇数范围

# nfz
DEPOT_MARGIN = 30.0      # 基地须远离任一禁飞圆边界 ≥30 m（2026-09-09 决策）
DEPOT_SEED = 20260909    # 重采样随机种子（确定性可复现）
TARGET_MARGIN = 30.0     # 目标点同样须远离任一禁飞圆边界 ≥30 m（2026-09-10 决策）
TARGET_SEED = 20260910   # 目标点重采样独立种子（不动基地重采样结果）

# dynamic
DYN_SEED = 1002
DYN_FRAC = 0.1
DYN_T_MIN, DYN_T_MAX = 100.0, 300.0

# dynamic 族与源逐位一致的共享字段（动态数据只多一个 dynamic_trigger_times）
SHARED_FIELDS = ("locs", "depot", "drone_types", "speed", "turning_radii",
                 "range_vals", "service_times", "drone_capabilities",
                 "nfz_centers", "nfz_radii", "num_nfz")


def _parse_scales(values, default):
    """解析规模列表：支持空格分隔与逗号分隔混用；未提供时用默认列表。"""
    if not values:
        return list(default)
    out = []
    for v in values:
        out.extend(s.strip() for s in str(v).split(',') if s.strip())
    return out


# ---------------------------------------------------------------------------
# 1) randdepot —— 主基准：随机基地
# ---------------------------------------------------------------------------
def gen_randdepot(data_dir, scales):
    """生成"出发点随机"的数据：depot 从坐标原点改为区域内任意位置。

    目标点 (locs)、无人机参数与 NFZ 均与原 *_real.npz 完全一致，仅把每个实例
    的 depot 改为区域内（[0, REGION]^2）均匀随机位置，以便隔离"出发点位置"变量。
    归一化参考尺度 compute_refs 只依赖 locs，不受 depot 影响，与原始数据可比。
    """
    rng = np.random.default_rng(SEED_RANDDEPOT)
    for scale in scales:
        src = os.path.join(data_dir, f"{scale}_real.npz")
        if not os.path.exists(src):
            print(f"跳过（源不存在）: {src}")
            continue
        d = np.load(src)
        n_inst = d["locs"].shape[0]

        # 区域内均匀随机出发点（每实例一个，互不相同）
        new_depot = rng.uniform(0.0, REGION, size=(n_inst, 2)).astype(np.float32)

        out = os.path.join(data_dir, f"{scale}_randdepot_real.npz")
        kwargs = {k: d[k] for k in d.files if k != "depot"}
        kwargs["depot"] = new_depot
        np.savez_compressed(out, **kwargs)
        print(f"已生成 {os.path.basename(out)}  {n_inst} 实例, depot 前3: "
              f"{[tuple(round(x,1) for x in r) for r in new_depot[:3]]}")


# ---------------------------------------------------------------------------
# 2) nonuni —— 非均匀目标分布 + 随机出发点
# ---------------------------------------------------------------------------
def _sample_cluster_points(rng, K, N):
    """在 [0,REGION]^2 内生成 N 个目标：K 个簇 + 少量均匀噪声点。

    返回 float32 (N,2)。簇中心在 [0.15,0.85]*REGION 内采样（留边距防截断出界），
    簇内点围绕中心按标准差 scale=REGION/(K+3) 的高斯采样并拒绝到区域外；
    各簇容量由 Dirichlet(alpha=2*K) 抽取后 round+adjust 保证总和 = N - n_noise。
    """
    n_noise = max(1, int(round(NOISE_FRAC * N)))
    n_clustered = N - n_noise

    # 簇中心
    centers = rng.uniform(0.15 * REGION, 0.85 * REGION, size=(K, 2))
    scale = REGION / (K + 3.0)

    # 各簇容量（Dirichlet 平滑，避免过大的空/单点簇）
    alpha = np.full(K, 2.0 * K)
    sizes = rng.dirichlet(alpha) * n_clustered
    sizes = np.round(sizes).astype(int)
    # 修正舍入误差使总和恰为 n_clustered
    diff = n_clustered - sizes.sum()
    if diff > 0:
        sizes[:diff] += 1
    elif diff < 0:
        sizes[:(-diff)] -= 1
    sizes = np.clip(sizes, 1, None)      # 每簇至少 1 点
    # 再次修正（clip 可能破坏总和）
    diff = n_clustered - sizes.sum()
    if diff > 0:
        # 把差额加到容量最大的簇上，保证总数
        idx = int(np.argmax(sizes))
        sizes[idx] += diff
    elif diff < 0:
        # 从容量最大的簇扣减
        idx = int(np.argmax(sizes))
        sizes[idx] += diff

    pts = []
    for c, s in zip(centers, sizes):
        # 拒绝采样保证在区域内的近似高斯团
        n_need = int(s)
        acc = []
        while len(acc) < n_need:
            cand = rng.normal(loc=c, scale=scale, size=(n_need * 4, 2))
            inside = (cand >= 0).all(axis=1) & (cand <= REGION).all(axis=1)
            acc.extend(cand[inside][: n_need - len(acc)])
        pts.append(np.asarray(acc[:n_need], dtype=np.float64))

    cluster = np.concatenate(pts, axis=0) if pts else np.empty((0, 2))
    # 噪声点（区域内均匀）
    noise = rng.uniform(0.0, REGION, size=(n_noise, 2))
    locs = np.concatenate([cluster, noise], axis=0).astype(np.float32)
    # 打乱顺序（避免算法利用"簇内目标相邻"这一顺序先验）
    perm = rng.permutation(N)
    return locs[perm]


def gen_nonuni(data_dir, scales):
    """生成"非均匀目标分布 + 随机出发点"的数据。

    仅重写 locs 与 depot，其余字段（drone_types/speed/turning_radii/range_vals/
    service_times/drone_capabilities/nfz）与原 *_real.npz 完全一致，
    以隔离"目标空间分布"这一变量。
    """
    rng = np.random.default_rng(SEED_NONUNI)
    for scale in scales:
        src = os.path.join(data_dir, f"{scale}_real.npz")
        if not os.path.exists(src):
            print(f"跳过（源不存在）: {src}")
            continue
        d = np.load(src)
        n_inst = d["locs"].shape[0]
        N = d["locs"].shape[1]

        new_locs = np.empty((n_inst, N, 2), dtype=np.float32)
        for i in range(n_inst):
            K = rng.integers(K_MIN, K_MAX + 1)
            new_locs[i] = _sample_cluster_points(rng, K, N)
        new_depot = rng.uniform(0.0, REGION, size=(n_inst, 2)).astype(np.float32)

        out = os.path.join(data_dir, f"{scale}_nonuni_randdepot_real.npz")
        kwargs = {k: d[k] for k in d.files if k not in ("locs", "depot")}
        kwargs["locs"] = new_locs
        kwargs["depot"] = new_depot
        np.savez_compressed(out, **kwargs)

        # 自检：范围内、每实例数量=N、depot 区域正确
        ok_range = bool((new_locs >= 0).all() and (new_locs <= REGION).all())
        print(f"已生成 {os.path.basename(out)}  {n_inst} 实例, N={N}, "
              f"范围正确={ok_range}, depot前3: {[tuple(round(x,1) for x in r) for r in new_depot[:3]]}")
        assert ok_range, f"{out} 目标越界!"
        assert new_depot.shape == (n_inst, 2), f"{out} depot 形状错误"


# ---------------------------------------------------------------------------
# 3) nfz —— 禁飞区扩展实验数据（更小转弯半径 + 基地/目标留净空）
# ---------------------------------------------------------------------------
def _resample_depots_away_nfz(depots, nfz_centers, nfz_radii, num_nfz,
                              margin=DEPOT_MARGIN, region=REGION, seed=DEPOT_SEED):
    """全量重新采样基地：均匀 [0,region]^2 内拒绝采样，使每个基地到任一禁飞圆
    边界距离 ≥ margin（圆外留 margin 安全距离）。确定性 default_rng(seed)。"""
    rng = np.random.default_rng(seed)
    n = depots.shape[0]
    out = np.empty_like(depots)
    for i in range(n):
        cs = np.asarray(nfz_centers[i])
        rs = np.asarray(nfz_radii[i], dtype=float)
        nz = int(num_nfz[i]) if num_nfz is not None else len(rs)
        for _ in range(200000):
            p = rng.uniform(0.0, region, size=2)
            ok = True
            for j in range(nz):
                if np.hypot(p[0] - cs[j][0], p[1] - cs[j][1]) - float(rs[j]) < margin:
                    ok = False
                    break
            if ok:
                out[i] = p.astype(out.dtype)
                break
        else:
            raise RuntimeError(f"depot 重采样失败：instance {i}（区域过密，可降 margin 或升 region）")
    return out


def _resample_targets_away_nfz(locs, nfz_centers, nfz_radii, num_nfz,
                               margin=TARGET_MARGIN, region=REGION, seed=TARGET_SEED):
    """把距任一禁飞圆边界 < margin 的目标点就地重采样到 ≥ margin 处（其余目标点不动）。
    独立种子，不改变基地重采样结果。确定性 default_rng(seed)。"""
    rng = np.random.default_rng(seed)
    out = np.array(locs, dtype=float)
    n = out.shape[0]
    moved = 0
    for i in range(n):
        cs = np.asarray(nfz_centers[i], dtype=float)
        rs = np.asarray(nfz_radii[i], dtype=float)
        nz = int(num_nfz[i]) if num_nfz is not None else len(rs)
        cs, rs = cs[:nz], rs[:nz]
        for k in range(out.shape[1]):
            if np.all(np.hypot(out[i, k, 0] - cs[:, 0], out[i, k, 1] - cs[:, 1]) - rs >= margin):
                continue
            for _ in range(200000):
                p = rng.uniform(0.0, region, size=2)
                if np.all(np.hypot(p[0] - cs[:, 0], p[1] - cs[:, 1]) - rs >= margin):
                    out[i, k] = p.astype(out.dtype)
                    moved += 1
                    break
            else:
                raise RuntimeError(
                    f"目标点重采样失败：instance {i} target {k}（区域过密，可降 margin 或升 region）")
    return out, moved


def _patch_nfz(src_path, out_path, turning_radius=10.0, resample_depots=False,
               depot_margin=DEPOT_MARGIN, depot_seed=DEPOT_SEED, service_time=0.0):
    d = np.load(src_path, allow_pickle=True)
    out = {k: d[k] for k in d.files}
    tr = np.asarray(d["turning_radii"])
    out["turning_radii"] = np.full_like(tr, float(turning_radius))
    if "service_times" in out:
        st = np.asarray(d["service_times"])
        if service_time is None:
            svc_desc = float(np.asarray(st, dtype=float).mean()) if st.size else 0.0
        else:
            out["service_times"] = np.full_like(st, float(service_time))
            svc_desc = float(service_time)
    else:
        svc_desc = None
    info = (f"turning_radius={turning_radius}, "
            f"service_times={'kept=' + format(svc_desc, 'g') if svc_desc is not None else 'absent'}")
    if resample_depots:
        if "nfz_centers" in out and "nfz_radii" in out:
            num_nfz = out.get("num_nfz") if "num_nfz" in out else None
            out["depot"] = _resample_depots_away_nfz(
                np.asarray(out["depot"]), out["nfz_centers"], out["nfz_radii"], num_nfz,
                margin=depot_margin, seed=depot_seed)
            info += f", depots resampled >={depot_margin:g}m from NFZ (seed {depot_seed})"
            # 目标点同样留净空：R=10m 的终端转弯弧扫描半径达 2R=20m，目标点贴着
            # 禁飞圆时该转弯必然扫入圆内，任何切线构造都无法消除，故须在场景层排除。
            out["locs"], _n_moved = _resample_targets_away_nfz(
                np.asarray(out["locs"]), out["nfz_centers"], out["nfz_radii"], num_nfz,
                margin=TARGET_MARGIN, seed=TARGET_SEED)
            info += f", targets resampled >={TARGET_MARGIN:g}m from NFZ ({_n_moved} moved, seed {TARGET_SEED})"
        else:
            print(f"WARN: {src_path} 无 nfz 字段，depot/targets 保持原样")
    np.savez_compressed(out_path, **out)
    print(f"OK: {out_path}  {info}")


def gen_nfz(data_dir, scales, turning_radius=10.0, service_time=0.0, randdepot=True):
    """为 NFZ 扩展实验生成定制数据。

    · turning_radii 全部设为 turning_radius（默认 10；原数据为 30，太大使 Dubins
      路径在障碍场景下易擦碰/绕飞过度）；
    · service_times 默认置 0（与已发布的 NFZ 族一致；Dubins 下由绕飞项计时，
      用 --keep-service-time 可保留源值）；
    · 默认从 {scale}_randdepot_real.npz 派生（需先生成 randdepot 族）。
    """
    suffix = "_randdepot" if randdepot else ""
    for s in scales:
        src = os.path.join(data_dir, f"{s}{suffix}_real.npz")
        out = os.path.join(data_dir, f"{s}{suffix}_nfz_real.npz")
        if not os.path.exists(src):
            print(f"SKIP: 缺 {src}")
            continue
        _patch_nfz(src, out, turning_radius, resample_depots=randdepot,
                   service_time=service_time)


# ---------------------------------------------------------------------------
# 4) dynamic —— 动态目标族（在线出现时间）
# ---------------------------------------------------------------------------
def add_dynamic(src_path, out_path, n_dyn_frac=DYN_FRAC, seed=DYN_SEED,
                t_min=DYN_T_MIN, t_max=DYN_T_MAX):
    d = np.load(src_path)
    missing = [f for f in SHARED_FIELDS if f not in d.files]
    if missing:
        raise SystemExit(f"[FAIL] {src_path} 缺字段 {missing}")

    # ---- 口径守卫：服务时间必须与主实验一致（非零）----
    svc = np.asarray(d["service_times"], dtype=float)
    svc_mean = float(svc.mean()) if svc.size else 0.0
    if svc_mean <= 0:
        raise SystemExit(
            f"[FAIL] {src_path} 的 service_times 均值为 {svc_mean:g} ≤ 0。\n"
            f"       动态场景的仿真口径必须与主实验一致（服务时间 5 s/子任务），\n"
            f"       NFZ 数据族的 service_times 为 0，不能作为动态数据的源。")

    S = d["locs"].shape[0]
    N = d["locs"].shape[1]
    n_dyn = max(1, int(round(N * n_dyn_frac)))
    rng = np.random.RandomState(seed)
    # 出现时间：末尾 n_dyn 个目标的侦察子任务(T1)在 [t_min, t_max] 内随机出现
    trig = np.zeros((S, N * 3), dtype=np.float32)
    for si in range(S):
        for t in range(N - n_dyn, N):
            trig[si, t * 3 + 0] = rng.uniform(t_min, t_max)
    np.savez_compressed(out_path, **{f: d[f] for f in SHARED_FIELDS},
                        dynamic_trigger_times=trig)

    # ---- 写盘后回读比对：共享字段必须与源逐位一致 ----
    chk = np.load(out_path)
    bad = [f for f in SHARED_FIELDS
           if not np.array_equal(np.asarray(chk[f]), np.asarray(d[f]))]
    if bad:
        raise SystemExit(f"[FAIL] {out_path} 的共享字段与源不一致: {bad}")

    print(f"OK: {out_path}  ({S}样本, {N}目标, 动态{n_dyn}个/样本, "
          f"出现时间 {t_min:.0f}~{t_max:.0f}s) | 口径: 源={os.path.basename(src_path)} "
          f"τs={svc_mean:g}s × {svc.size} 项 | 共享字段与源逐位一致")


def gen_dynamic(data_dir, scales, n_dyn_frac=DYN_FRAC, seed=DYN_SEED,
                t_min=DYN_T_MIN, t_max=DYN_T_MAX):
    """生成含动态目标的随机基地尺度数据（在 {scale}_randdepot_real.npz 上
    加 dynamic_trigger_times）。

    动态目标模型（在线重规划场景）：把末尾 N_dyn 个目标标记为"执行中随机出现"，
    为其侦察子任务(T1)设置一个出现时间（[t_min, t_max]），T2/T3 出现时间为 0。
    """
    for s in scales:
        src = os.path.join(data_dir, f"{s}_randdepot_real.npz")
        out = os.path.join(data_dir, f"{s}_randdepot_dyn_real.npz")
        if not os.path.exists(src):
            print(f"SKIP: 缺 {src}")
            continue
        add_dynamic(src, out, n_dyn_frac, seed, t_min, t_max)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    common = argparse.ArgumentParser(add_help=False)
    # 默认 SUPPRESS：避免子解析器的默认值覆盖主解析器已读到的值，
    # 使 --data-dir 放在子命令前（gen_benchmark_data.py --data-dir D randdepot）
    # 或子命令后（gen_benchmark_data.py randdepot --data-dir D）都可用。
    common.add_argument("--data-dir", default=argparse.SUPPRESS,
                        help="数据目录（默认：本脚本上一级的 data/；可放在子命令前或后）")

    ap = argparse.ArgumentParser(
        description="数值实验基准数据生成器（randdepot | nonuni | nfz | dynamic | all）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        parents=[common],
        epilog=__doc__)
    sub = ap.add_subparsers(dest="family", required=True)

    p_r = sub.add_parser("randdepot", parents=[common], help="主基准：随机基地")
    p_r.add_argument("--scales", nargs="*", default=None,
                     help="规模列表（逗号/空格分隔），默认全部六档")

    p_n = sub.add_parser("nonuni", parents=[common], help="非均匀目标布局（高斯混合簇 + 10%% 噪声）")
    p_n.add_argument("--scales", nargs="*", default=None,
                     help="规模列表（逗号/空格分隔），默认全部六档")

    p_f = sub.add_parser("nfz", parents=[common], help="NFZ 族（转弯半径 10 m，基地/目标留净空）")
    p_f.add_argument("--scales", nargs="*", default=None,
                     help="规模列表（逗号/空格分隔），默认 n30_m6 n60_m12 n100_m16 n150_m24 n200_m32")
    p_f.add_argument("--turning-radius", type=float, default=10.0, help="最小转弯半径（米，默认 10）")
    p_f.add_argument("--service-time", type=float, default=0.0,
                     help="覆盖 service_times（默认 0，与已发布 NFZ 族一致）")
    p_f.add_argument("--keep-service-time", action="store_true",
                     help="保留源数据的 service_times（不置 0）")
    p_f.add_argument("--no-randdepot", action="store_false", dest="randdepot",
                     help="从 {scale}_real.npz 而非 {scale}_randdepot_real.npz 派生")

    p_d = sub.add_parser("dynamic", parents=[common], help="动态目标族（末尾 10%% 目标在线出现）")
    p_d.add_argument("--scales", nargs="*", default=None,
                     help="规模列表（逗号/空格分隔，无需带 _randdepot 后缀），默认 n60_m12 n100_m16 n150_m24 n200_m32")
    p_d.add_argument("--n-dyn-frac", type=float, default=DYN_FRAC, help="动态目标占比（默认 0.1）")
    p_d.add_argument("--seed", type=int, default=DYN_SEED, help="随机种子（默认 1002）")
    p_d.add_argument("--t-min", type=float, default=DYN_T_MIN, help="出现时间下界（秒，默认 100）")
    p_d.add_argument("--t-max", type=float, default=DYN_T_MAX, help="出现时间上界（秒，默认 300）")

    p_a = sub.add_parser("all", parents=[common], help="依次生成全部四族（默认规模）")

    a = ap.parse_args()
    data_dir = getattr(a, "data_dir", None) or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
    data_dir = os.path.abspath(data_dir)
    os.makedirs(data_dir, exist_ok=True)
    print(f"[gen_benchmark_data] data-dir = {data_dir}")

    if a.family == "randdepot":
        gen_randdepot(data_dir, _parse_scales(a.scales, DEFAULT_SCALES))
    elif a.family == "nonuni":
        gen_nonuni(data_dir, _parse_scales(a.scales, DEFAULT_SCALES))
    elif a.family == "nfz":
        gen_nfz(data_dir, _parse_scales(a.scales, DEFAULT_NFZ_SCALES),
                turning_radius=a.turning_radius,
                service_time=None if a.keep_service_time else a.service_time,
                randdepot=a.randdepot)
    elif a.family == "dynamic":
        gen_dynamic(data_dir, _parse_scales(a.scales, DEFAULT_DYN_SCALES),
                    n_dyn_frac=a.n_dyn_frac, seed=a.seed, t_min=a.t_min, t_max=a.t_max)
    elif a.family == "all":
        print("== [1/4] randdepot =="); gen_randdepot(data_dir, DEFAULT_SCALES)
        print("== [2/4] nonuni ==");    gen_nonuni(data_dir, DEFAULT_SCALES)
        print("== [3/4] nfz ==");       gen_nfz(data_dir, DEFAULT_NFZ_SCALES)
        print("== [4/4] dynamic ==");   gen_dynamic(data_dir, DEFAULT_DYN_SCALES)

    print("[gen_benchmark_data] 完成")


if __name__ == "__main__":
    main()
