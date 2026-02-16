import csv
from pathlib import Path


def read_rpm_distance(path: Path):
    rpm = []
    dist = []
    time_at_hub = []
    max_height = []
    hub_height = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rpm.append(float(row["rpm"]))
            dist.append(float(row["distance_at_hub_height_m"]))
            time_at_hub.append(float(row.get("time_at_hub_height_s", "0") or 0.0))
            max_height.append(float(row.get("max_height_m", "0") or 0.0))
            hub_height.append(float(row.get("hub_height_m", "0") or 0.0))
    hub_h = hub_height[0] if hub_height and hub_height[0] > 0.0 else None
    return rpm, dist, time_at_hub, max_height, hub_h


def read_trajectories(path: Path):
    data = {}
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rpm = round(float(row["rpm"]), 6)
            x = float(row["x_m"])
            z = float(row["z_m"])
            if rpm not in data:
                data[rpm] = {"x": [], "z": []}
            data[rpm]["x"].append(x)
            data[rpm]["z"].append(z)
    return data


def read_direction_sweep(path: Path):
    data = {}
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            speed = round(float(row["speed_mps"]), 6)
            entry = data.setdefault(
                speed,
                {
                    "direction_deg": [],
                    "turret_yaw_deg": [],
                    "top_rpm": [],
                    "bottom_rpm": [],
                    "closest_error_m": [],
                    "raw_scale": [],
                    "applied_scale": [],
                    "required_muzzle_speed_mps": [],
                    "rpm_saturated_pct": [],
                },
            )
            entry["direction_deg"].append(float(row["direction_deg"]))
            entry["turret_yaw_deg"].append(float(row["turret_yaw_deg"]))
            entry["top_rpm"].append(float(row["top_rpm"]))
            entry["bottom_rpm"].append(float(row["bottom_rpm"]))
            entry["closest_error_m"].append(float(row["closest_error_m"]))
            entry["raw_scale"].append(float(row.get("raw_scale", "1.0") or 1.0))
            entry["applied_scale"].append(float(row.get("applied_scale", "1.0") or 1.0))
            entry["required_muzzle_speed_mps"].append(
                float(row.get("required_muzzle_speed_mps", row.get("muzzle_speed_mps", "0.0")) or 0.0)
            )
            saturated = row.get("rpm_saturated_any", row.get("scale_clamped", "false"))
            entry["rpm_saturated_pct"].append(100.0 if saturated.lower() == "true" else 0.0)
    return data


def read_robustness_grid(path: Path):
    rows = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            sat = row.get("rpm_saturated_any", row.get("scale_clamped", "false"))
            rows.append(
                {
                    "distance_m": float(row["distance_m"]),
                    "speed_mps": float(row["speed_mps"]),
                    "closest_error_m": float(row["closest_error_m"]),
                    "hit": (row["hit"].lower() == "true"),
                    "rpm_saturated_any": (sat.lower() == "true"),
                }
            )
    return rows


def read_focus_summary(path: Path):
    if not path.exists():
        return []
    rows = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(
                {
                    "scope": row["scope"],
                    "samples": int(row["samples"]),
                    "hit_rate_pct": float(row["hit_rate_pct"]),
                    "mean_closest_error_m": float(row["mean_closest_error_m"]),
                    "rpm_saturation_rate_pct": float(row["rpm_saturation_rate_pct"]),
                    "solve_invalid_rate_pct": float(row["solve_invalid_rate_pct"]),
                }
            )
    return rows


def sign(x):
    if x > 0:
        return 1.0
    if x < 0:
        return -1.0
    return 0.0


def detect_threshold_rpm(rpm, dist, time_at_hub, max_height, hub_height):
    """Find first valid hub-height crossing RPM from sampled data."""
    first_hit_idx = None
    for i, (d, t) in enumerate(zip(dist, time_at_hub)):
        if d > 0.0 or t > 0.0:
            first_hit_idx = i
            break
    if first_hit_idx is None or first_hit_idx == 0:
        return rpm[0], 0, 0
    return rpm[first_hit_idx], first_hit_idx, first_hit_idx - 1


def linear_interp(xs, ys, x):
    if not xs:
        return 0.0
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    i = 0
    while i + 1 < len(xs) and xs[i + 1] < x:
        i += 1
    x0, x1 = xs[i], xs[i + 1]
    y0, y1 = ys[i], ys[i + 1]
    if abs(x1 - x0) < 1e-12:
        return y0
    a = (x - x0) / (x1 - x0)
    return y0 + a * (y1 - y0)


def pchip_slopes(xs, ys):
    """Monotone PCHIP slopes (Fritsch-Carlson)."""
    n = len(xs)
    if n < 2:
        return [0.0] * n
    h = [xs[i + 1] - xs[i] for i in range(n - 1)]
    d = [(ys[i + 1] - ys[i]) / h[i] if h[i] != 0 else 0.0 for i in range(n - 1)]
    m = [0.0] * n
    if n == 2:
        m[0] = d[0]
        m[1] = d[0]
        return m

    for i in range(1, n - 1):
        if d[i - 1] * d[i] <= 0.0:
            m[i] = 0.0
        else:
            w1 = 2.0 * h[i] + h[i - 1]
            w2 = h[i] + 2.0 * h[i - 1]
            m[i] = (w1 + w2) / ((w1 / d[i - 1]) + (w2 / d[i]))

    m0 = ((2.0 * h[0] + h[1]) * d[0] - h[0] * d[1]) / (h[0] + h[1])
    if sign(m0) != sign(d[0]):
        m0 = 0.0
    elif sign(d[0]) != sign(d[1]) and abs(m0) > abs(3.0 * d[0]):
        m0 = 3.0 * d[0]
    m[0] = m0

    mn = ((2.0 * h[-1] + h[-2]) * d[-1] - h[-1] * d[-2]) / (h[-1] + h[-2])
    if sign(mn) != sign(d[-1]):
        mn = 0.0
    elif sign(d[-1]) != sign(d[-2]) and abs(mn) > abs(3.0 * d[-1]):
        mn = 3.0 * d[-1]
    m[-1] = mn
    return m


def pchip_eval(xs, ys, ms, x):
    n = len(xs)
    if n == 0:
        return 0.0
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]

    i = 0
    while i + 1 < n and xs[i + 1] < x:
        i += 1
    h = xs[i + 1] - xs[i]
    if h <= 0.0:
        return ys[i]
    t = (x - xs[i]) / h
    t2 = t * t
    t3 = t2 * t
    h00 = 2.0 * t3 - 3.0 * t2 + 1.0
    h10 = t3 - 2.0 * t2 + t
    h01 = -2.0 * t3 + 3.0 * t2
    h11 = t3 - t2
    return h00 * ys[i] + h10 * h * ms[i] + h01 * ys[i + 1] + h11 * h * ms[i + 1]


def build_approx_eval(threshold_rpm, first_active_idx, rpm, dist):
    active_rpm = rpm[first_active_idx:]
    active_dist = dist[first_active_idx:]
    active_slopes = pchip_slopes(active_rpm, active_dist)
    first_active_rpm = active_rpm[0] if active_rpm else threshold_rpm
    first_active_dist = active_dist[0] if active_dist else 0.0

    def f(x):
        if x < threshold_rpm:
            return 0.0
        if x <= first_active_rpm:
            if first_active_rpm - threshold_rpm < 1e-12:
                return first_active_dist
            a = (x - threshold_rpm) / (first_active_rpm - threshold_rpm)
            return a * first_active_dist
        return max(0.0, pchip_eval(active_rpm, active_dist, active_slopes, x))

    return f


def optimize_threshold(rpm, dist, first_active_idx, lo_idx):
    # Minimize L1 area between approximation and piecewise-linear simulation curve.
    lo = rpm[lo_idx]
    hi = rpm[first_active_idx]
    best_t = hi
    best_cost = float("inf")
    for k in range(201):
        t = lo + (hi - lo) * k / 200.0
        approx = build_approx_eval(t, first_active_idx, rpm, dist)
        cost = 0.0
        n = 600
        x_prev = rpm[0]
        e_prev = abs(approx(x_prev) - linear_interp(rpm, dist, x_prev))
        for i in range(1, n + 1):
            x = rpm[0] + (rpm[-1] - rpm[0]) * i / n
            e = abs(approx(x) - linear_interp(rpm, dist, x))
            cost += 0.5 * (e_prev + e) * (x - x_prev)
            x_prev = x
            e_prev = e
        if cost < best_cost:
            best_cost = cost
            best_t = t
    return best_t


def scale_point(x, y, x_min, x_max, y_min, y_max, width, height, pad):
    px = pad + (x - x_min) / (x_max - x_min) * (width - 2 * pad)
    py = height - pad - (y - y_min) / (y_max - y_min) * (height - 2 * pad)
    return px, py


def draw_axes(svg, width, height, pad, x_min, x_max, y_min, y_max, xlabel, ylabel):
    svg.append(f'<rect x="0" y="0" width="{width}" height="{height}" fill="white" stroke="none"/>')
    svg.append(
        f'<rect x="{pad}" y="{pad}" width="{width - 2 * pad}" height="{height - 2 * pad}" fill="none" stroke="#444" stroke-width="1"/>'
    )
    ticks = 8
    for i in range(ticks + 1):
        tx = x_min + (x_max - x_min) * i / ticks
        px, _ = scale_point(tx, y_min, x_min, x_max, y_min, y_max, width, height, pad)
        svg.append(f'<line x1="{px:.2f}" y1="{pad}" x2="{px:.2f}" y2="{height - pad}" stroke="#ddd" stroke-width="1"/>')
        svg.append(f'<text x="{px:.2f}" y="{height - pad + 18}" font-size="12" text-anchor="middle" fill="#222">{tx:.1f}</text>')
    for i in range(ticks + 1):
        ty = y_min + (y_max - y_min) * i / ticks
        _, py = scale_point(x_min, ty, x_min, x_max, y_min, y_max, width, height, pad)
        svg.append(f'<line x1="{pad}" y1="{py:.2f}" x2="{width - pad}" y2="{py:.2f}" stroke="#ddd" stroke-width="1"/>')
        svg.append(f'<text x="{pad - 8}" y="{py + 4:.2f}" font-size="12" text-anchor="end" fill="#222">{ty:.1f}</text>')
    svg.append(f'<text x="{width / 2:.2f}" y="{height - 10}" font-size="14" text-anchor="middle" fill="#111">{xlabel}</text>')
    svg.append(
        f'<text x="18" y="{height / 2:.2f}" font-size="14" text-anchor="middle" transform="rotate(-90 18 {height / 2:.2f})" fill="#111">{ylabel}</text>'
    )


def write_rpm_distance_svg(path: Path, rpm, dist, time_at_hub, max_height, hub_height, zoom=False):
    width, height, pad = 1200, 520, 60
    x_min, x_max = min(rpm), max(rpm)
    y_min, y_max = min(dist), max(dist)
    y_pad = 0.1 * (y_max - y_min if y_max > y_min else 1.0)
    y_min -= y_pad
    y_max += y_pad

    threshold_detected, first_active_idx, lo_idx = detect_threshold_rpm(rpm, dist, time_at_hub, max_height, hub_height)
    if first_active_idx > 0:
        threshold_rpm = optimize_threshold(rpm, dist, first_active_idx, lo_idx)
    else:
        threshold_rpm = threshold_detected
    approx = build_approx_eval(threshold_rpm, first_active_idx, rpm, dist)

    if zoom and first_active_idx > 0:
        x_min = max(rpm[0], rpm[lo_idx] - 180.0)
        x_max = min(rpm[-1], rpm[first_active_idx] + 220.0)
        y_min = -0.05
        y_max = max(dist[first_active_idx] * 1.25, 0.8)

    approx_xy = []
    for i in range(0, 501):
        x = x_min + (x_max - x_min) * i / 500
        approx_xy.append((x, approx(x)))

    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">']
    draw_axes(svg, width, height, pad, x_min, x_max, y_min, y_max, "Shooter RPM", "Distance [m]")

    sim_points = []
    for x, y in zip(rpm, dist):
        px, py = scale_point(x, y, x_min, x_max, y_min, y_max, width, height, pad)
        sim_points.append(f"{px:.2f},{py:.2f}")
    svg.append(f'<polyline fill="none" stroke="#1f77b4" stroke-width="2" points="{" ".join(sim_points)}"/>')

    fit_points = []
    for x, y in approx_xy:
        px, py = scale_point(x, y, x_min, x_max, y_min, y_max, width, height, pad)
        fit_points.append(f"{px:.2f},{py:.2f}")
    svg.append(f'<polyline fill="none" stroke="#ff7f0e" stroke-width="2" points="{" ".join(fit_points)}"/>')

    tx, _ = scale_point(threshold_rpm, y_min, x_min, x_max, y_min, y_max, width, height, pad)
    svg.append(
        f'<line x1="{tx:.2f}" y1="{pad}" x2="{tx:.2f}" y2="{height - pad}" stroke="#999" stroke-width="1.2" stroke-dasharray="6,4"/>'
    )

    svg.append('<rect x="70" y="20" width="370" height="62" fill="white" stroke="#aaa" stroke-width="1"/>')
    svg.append('<line x1="84" y1="36" x2="124" y2="36" stroke="#1f77b4" stroke-width="2"/>')
    svg.append('<text x="132" y="40" font-size="14" fill="#111">simulation</text>')
    svg.append('<line x1="84" y1="54" x2="124" y2="54" stroke="#ff7f0e" stroke-width="2"/>')
    svg.append('<text x="132" y="58" font-size="14" fill="#111">piecewise monotone spline</text>')
    svg.append(f'<text x="84" y="76" font-size="13" fill="#333">rpm_threshold ~= {threshold_rpm:.1f}</text>')

    svg.append("</svg>")
    path.write_text("\n".join(svg), encoding="utf-8")


def write_trajectories_svg(path: Path, trajectories):
    width, height, pad = 1200, 520, 60
    x_max = 0.0
    y_max = 0.0
    for tr in trajectories.values():
        if tr["x"]:
            x_max = max(x_max, max(tr["x"]))
        if tr["z"]:
            y_max = max(y_max, max(tr["z"]))

    x_min, y_min = 0.0, 0.0
    x_max *= 1.03
    y_max *= 1.08

    colors = [
        "#1f77b4",
        "#ff7f0e",
        "#2ca02c",
        "#d62728",
        "#9467bd",
        "#8c564b",
        "#e377c2",
        "#7f7f7f",
        "#bcbd22",
        "#17becf",
    ]

    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">']
    draw_axes(svg, width, height, pad, x_min, x_max, y_min, y_max, "Distance [m]", "Height [m]")

    for i, rpm in enumerate(sorted(trajectories.keys())):
        tr = trajectories[rpm]
        points = []
        for x, z in zip(tr["x"], tr["z"]):
            px, py = scale_point(x, z, x_min, x_max, y_min, y_max, width, height, pad)
            points.append(f"{px:.2f},{py:.2f}")
        svg.append(
            f'<polyline fill="none" stroke="{colors[i % len(colors)]}" stroke-width="1.4" points="{" ".join(points)}"/>'
        )

    svg.append("</svg>")
    path.write_text("\n".join(svg), encoding="utf-8")


def write_grouped_lines_svg(path: Path, grouped_data, x_key, y_key, title, x_label, y_label, y_pad_ratio=0.08):
    width, height, pad = 1200, 520, 60
    x_values = []
    for group in grouped_data.values():
        x_values.extend(group[x_key])
    if not x_values:
        x_values = [0.0, 1.0]
    x_min = min(x_values)
    x_max = max(x_values)
    if abs(x_max - x_min) < 1e-9:
        x_max = x_min + 1.0

    y_values = []
    for speed in grouped_data:
        y_values.extend(grouped_data[speed][y_key])
    if not y_values:
        y_values = [0.0]
    y_min = min(y_values)
    y_max = max(y_values)
    if abs(y_max - y_min) < 1e-9:
        y_max = y_min + 1.0
    y_pad = (y_max - y_min) * y_pad_ratio
    y_min -= y_pad
    y_max += y_pad

    colors = [
        "#1f77b4",
        "#ff7f0e",
        "#2ca02c",
        "#d62728",
        "#9467bd",
        "#8c564b",
    ]

    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">']
    draw_axes(svg, width, height, pad, x_min, x_max, y_min, y_max, x_label, y_label)
    svg.append(f'<text x="{width / 2:.2f}" y="24" font-size="16" text-anchor="middle" fill="#111">{title}</text>')

    sorted_speeds = sorted(grouped_data.keys())
    for i, speed in enumerate(sorted_speeds):
        d = grouped_data[speed]
        pairs = sorted(zip(d[x_key], d[y_key]), key=lambda t: t[0])
        points = []
        for x, y in pairs:
            px, py = scale_point(x, y, x_min, x_max, y_min, y_max, width, height, pad)
            points.append(f"{px:.2f},{py:.2f}")
        svg.append(
            f'<polyline fill="none" stroke="{colors[i % len(colors)]}" stroke-width="2" points="{" ".join(points)}"/>'
        )

    legend_h = 24 + 18 * len(sorted_speeds)
    svg.append(f'<rect x="76" y="26" width="220" height="{legend_h}" fill="white" stroke="#aaa" stroke-width="1"/>')
    for i, speed in enumerate(sorted_speeds):
        y = 42 + i * 18
        color = colors[i % len(colors)]
        svg.append(f'<line x1="90" y1="{y}" x2="130" y2="{y}" stroke="{color}" stroke-width="2"/>')
        svg.append(f'<text x="138" y="{y + 4}" font-size="13" fill="#111">speed={speed:.1f} m/s</text>')

    svg.append("</svg>")
    path.write_text("\n".join(svg), encoding="utf-8")


def write_robustness_summary_svgs(hit_path: Path, err_path: Path, clamp_path: Path, rows):
    grouped = {}
    for r in rows:
        key = (round(r["distance_m"], 3), round(r["speed_mps"], 3))
        grouped.setdefault(key, []).append(r)

    by_distance = {}
    for (distance, speed), vals in grouped.items():
        hit_rate = sum(1 for v in vals if v["hit"]) / len(vals)
        mean_err = sum(v["closest_error_m"] for v in vals) / len(vals)
        clamp_rate = sum(1 for v in vals if v["rpm_saturated_any"]) / len(vals)
        bucket = by_distance.setdefault(
            distance,
            {"speed_mps": [], "hit_rate_pct": [], "mean_error_m": [], "clamp_rate_pct": []},
        )
        bucket["speed_mps"].append(speed)
        bucket["hit_rate_pct"].append(hit_rate * 100.0)
        bucket["mean_error_m"].append(mean_err)
        bucket["clamp_rate_pct"].append(clamp_rate * 100.0)

    write_grouped_lines_svg(
        hit_path,
        by_distance,
        "speed_mps",
        "hit_rate_pct",
        "Robustness: hit rate vs speed",
        "Robot speed [m/s]",
        "Hit rate [%]",
        y_pad_ratio=0.04,
    )
    write_grouped_lines_svg(
        err_path,
        by_distance,
        "speed_mps",
        "mean_error_m",
        "Robustness: mean closest error vs speed",
        "Robot speed [m/s]",
        "Mean closest error [m]",
        y_pad_ratio=0.10,
    )
    write_grouped_lines_svg(
        clamp_path,
        by_distance,
        "speed_mps",
        "clamp_rate_pct",
        "Robustness: RPM saturation rate vs speed",
        "Robot speed [m/s]",
        "RPM saturation [%]",
        y_pad_ratio=0.04,
    )


def main():
    output_dir = Path("build/shot_plots")
    rpm_distance_csv = output_dir / "rpm_distance.csv"
    trajectories_csv = output_dir / "trajectories.csv"
    direction_sweep_csv = output_dir / "direction_sweep.csv"
    robustness_grid_csv = output_dir / "robustness_grid.csv"
    focus_summary_csv = output_dir / "robustness_focus_summary.csv"
    if (
        not rpm_distance_csv.exists()
        or not trajectories_csv.exists()
        or not direction_sweep_csv.exists()
        or not robustness_grid_csv.exists()
    ):
        raise FileNotFoundError("CSV files missing. Run `gradlew shotPlotDataExport` first.")

    rpm, dist, time_at_hub, max_height, hub_height = read_rpm_distance(rpm_distance_csv)
    trajectories = read_trajectories(trajectories_csv)
    direction_sweep = read_direction_sweep(direction_sweep_csv)
    robustness_rows = read_robustness_grid(robustness_grid_csv)
    focus_summary = read_focus_summary(focus_summary_csv)

    rpm_plot = output_dir / "rpm_distance_plot.svg"
    rpm_zoom_plot = output_dir / "rpm_distance_transition_zoom.svg"
    traj_plot = output_dir / "trajectories_plot.svg"
    turret_plot = output_dir / "turret_angle_vs_direction.svg"
    motor_plot = output_dir / "motor_rpm_vs_direction.svg"
    error_plot = output_dir / "closest_error_vs_direction.svg"
    scale_plot = output_dir / "scale_vs_direction.svg"
    saturation_plot = output_dir / "rpm_saturation_vs_direction.svg"
    robust_hit_plot = output_dir / "robust_hit_rate_vs_speed.svg"
    robust_err_plot = output_dir / "robust_mean_error_vs_speed.svg"
    robust_clamp_plot = output_dir / "robust_clamp_rate_vs_speed.svg"
    write_rpm_distance_svg(rpm_plot, rpm, dist, time_at_hub, max_height, hub_height)
    write_rpm_distance_svg(rpm_zoom_plot, rpm, dist, time_at_hub, max_height, hub_height, zoom=True)
    write_trajectories_svg(traj_plot, trajectories)
    write_grouped_lines_svg(
        turret_plot,
        direction_sweep,
        "direction_deg",
        "turret_yaw_deg",
        "Turret yaw vs robot velocity direction",
        "Robot velocity direction [deg]",
        "Turret yaw [deg]",
    )
    # Direction-dependent wheel-speed compensation overview.
    write_grouped_lines_svg(
        motor_plot,
        direction_sweep,
        "direction_deg",
        "top_rpm",
        "Top motor RPM vs robot velocity direction",
        "Robot velocity direction [deg]",
        "Top wheel RPM",
    )
    write_grouped_lines_svg(
        error_plot,
        direction_sweep,
        "direction_deg",
        "closest_error_m",
        "Closest shot error vs robot velocity direction",
        "Robot velocity direction [deg]",
        "Closest error [m]",
    )
    write_grouped_lines_svg(
        scale_plot,
        direction_sweep,
        "direction_deg",
        "required_muzzle_speed_mps",
        "Required muzzle speed vs robot velocity direction",
        "Robot velocity direction [deg]",
        "Required muzzle speed [m/s]",
        y_pad_ratio=0.05,
    )
    write_grouped_lines_svg(
        saturation_plot,
        direction_sweep,
        "direction_deg",
        "rpm_saturated_pct",
        "RPM saturation indicator vs robot velocity direction",
        "Robot velocity direction [deg]",
        "RPM saturated [%]",
        y_pad_ratio=0.05,
    )
    write_robustness_summary_svgs(
        robust_hit_plot,
        robust_err_plot,
        robust_clamp_plot,
        robustness_rows,
    )

    print("Saved plots:")
    print(f" - {rpm_plot.resolve()}")
    print(f" - {rpm_zoom_plot.resolve()}")
    print(f" - {traj_plot.resolve()}")
    print(f" - {turret_plot.resolve()}")
    print(f" - {motor_plot.resolve()}")
    print(f" - {error_plot.resolve()}")
    print(f" - {scale_plot.resolve()}")
    print(f" - {saturation_plot.resolve()}")
    print(f" - {robust_hit_plot.resolve()}")
    print(f" - {robust_err_plot.resolve()}")
    print(f" - {robust_clamp_plot.resolve()}")
    if focus_summary:
        print("Focus summary:")
        for row in focus_summary:
            print(
                f" - {row['scope']}: hit={row['hit_rate_pct']:.2f}% err={row['mean_closest_error_m']:.3f}m sat={row['rpm_saturation_rate_pct']:.2f}% invalid={row['solve_invalid_rate_pct']:.2f}%"
            )


if __name__ == "__main__":
    main()
