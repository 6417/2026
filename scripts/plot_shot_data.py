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


def main():
    output_dir = Path("build/shot_plots")
    rpm_distance_csv = output_dir / "rpm_distance.csv"
    trajectories_csv = output_dir / "trajectories.csv"
    if not rpm_distance_csv.exists() or not trajectories_csv.exists():
        raise FileNotFoundError("CSV files missing. Run `gradlew shotPlotDataExport` first.")

    rpm, dist, time_at_hub, max_height, hub_height = read_rpm_distance(rpm_distance_csv)
    trajectories = read_trajectories(trajectories_csv)

    rpm_plot = output_dir / "rpm_distance_plot.svg"
    rpm_zoom_plot = output_dir / "rpm_distance_transition_zoom.svg"
    traj_plot = output_dir / "trajectories_plot.svg"
    write_rpm_distance_svg(rpm_plot, rpm, dist, time_at_hub, max_height, hub_height)
    write_rpm_distance_svg(rpm_zoom_plot, rpm, dist, time_at_hub, max_height, hub_height, zoom=True)
    write_trajectories_svg(traj_plot, trajectories)

    print("Saved plots:")
    print(f" - {rpm_plot.resolve()}")
    print(f" - {rpm_zoom_plot.resolve()}")
    print(f" - {traj_plot.resolve()}")


if __name__ == "__main__":
    main()
