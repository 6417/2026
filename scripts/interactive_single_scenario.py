import json
import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from sys import platform


REPO_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_JSON = REPO_ROOT / "build" / "shot_plots" / "single_scenario_output.json"

if (platform == "win32"):
    GRADLEW = REPO_ROOT / "gradlew.bat"
else:
    GRADLEW = "gradle"

class ScenarioUI:
    def __init__(self):
        self.mode = "single"
        self.point_a = None
        self.point_b = None

        self.heading_deg = 0.0
        self.heading_step_deg = 5.0
        self.robot_speed_mps = 2.0

        self.curve_shot_count = 5
        self.curve_side_sign = 1.0

        self.fig, self.ax = plt.subplots(figsize=(11, 7))
        self.fig.canvas.manager.set_window_title("Single Scenario Shot Visualizer")
        self.cbar = None

        self._setup_axes()
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self._draw_status(self._help_text())

    def _setup_axes(self):
        self.ax.set_facecolor("white")
        self.ax.set_title("Whiteboard Shot Scenario (Blue Hub + Radius)")
        self.ax.set_xlabel("Field X [m]")
        self.ax.set_ylabel("Field Y [m]")
        self.ax.set_xlim(0.0, 16.8)
        self.ax.set_ylim(0.0, 8.2)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, color="#e5e5e5", linewidth=0.8)

    @staticmethod
    def _wrap_deg(angle_deg):
        return ((angle_deg + 180.0) % 360.0) - 180.0

    def _help_text(self):
        if self.mode == "single":
            return (
                "Mode=single | Click 1: robot position, Click 2: velocity direction | "
                "A/D heading -/+ | +/- speed | M toggle mode | R reset | Q quit"
            )
        return (
            "Mode=curve(5 shots) | Click 1: path start, Click 2: path end | "
            "+/- speed | V flip curve side | M toggle mode | R reset | Q quit"
        )

    def _draw_status(self, text):
        self.ax.text(
            0.02,
            0.98,
            text,
            transform=self.ax.transAxes,
            fontsize=9,
            va="top",
            ha="left",
            bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "#cccccc"},
            zorder=20,
        )

    def _run_single_scenario(self, x, y, heading_deg, vx, vy):
        args = f"{x:.6f},{y:.6f},{heading_deg:.6f},{vx:.6f},{vy:.6f},{OUTPUT_JSON.as_posix()}"
        cmd = [str(GRADLEW), "singleScenarioRun", f"-PsingleScenarioArgs={args}"]
        completed = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
        if completed.returncode != 0:
            stderr = completed.stderr.strip() or completed.stdout.strip() or "unknown error"
            raise RuntimeError(stderr)
        if not OUTPUT_JSON.exists():
            raise RuntimeError(f"missing output json: {OUTPUT_JSON}")
        return json.loads(OUTPUT_JSON.read_text(encoding="utf-8"))

    def _clear_plot_objects(self):
        if self.cbar is not None:
            try:
                self.cbar.remove()
            except Exception:
                cax = getattr(self.cbar, "ax", None)
                if cax is not None:
                    try:
                        cax.remove()
                    except Exception:
                        pass
            self.cbar = None
        self.ax.clear()
        self._setup_axes()

    @staticmethod
    def _compute_velocity_from_direction(dx, dy, speed_mps):
        direction_norm = np.hypot(dx, dy)
        if direction_norm < 1e-9:
            return 0.0, 0.0
        return (dx / direction_norm) * speed_mps, (dy / direction_norm) * speed_mps

    @staticmethod
    def _quadratic_bezier(p0, p1, p2, t):
        omt = 1.0 - t
        return (omt * omt * p0) + (2.0 * omt * t * p1) + (t * t * p2)

    @staticmethod
    def _quadratic_bezier_tangent(p0, p1, p2, t):
        return (2.0 * (1.0 - t) * (p1 - p0)) + (2.0 * t * (p2 - p1))
        

    def _compute_curve_control(self, p0, p2):
        seg = p2 - p0
        dist = np.linalg.norm(seg)
        if dist < 1e-9:
            return 0.5 * (p0 + p2)
        tangent = seg / dist
        normal = np.array([-tangent[1], tangent[0]])
        lateral = np.clip(0.35 * dist, 0.4, 2.0)
        return 0.5 * (p0 + p2) + (self.curve_side_sign * lateral * normal)

    def _draw_common_hub(self, data):
        hub = data["hub"]
        hub_center = (hub["x_m"], hub["y_m"])
        hub_radius = hub["radius_m"]
        self.ax.scatter([hub_center[0]], [hub_center[1]], c="blue", s=80, zorder=10, label="Hub Center")
        self.ax.add_patch(Circle(hub_center, hub_radius, fill=False, color="blue", linewidth=2.0, alpha=0.8))
        return hub_center

    def _draw_turret_arrows(self, launch_xy, hub_center, heading_deg, command, color_comp, color_novel):
        compensated_yaw_field_deg = heading_deg + command["turret_yaw_deg"]
        dx_hub = hub_center[0] - launch_xy[0]
        dy_hub = hub_center[1] - launch_xy[1]
        if np.hypot(dx_hub, dy_hub) < 1e-9:
            return np.nan, np.nan

        no_vel_yaw_field_deg = np.degrees(np.arctan2(dy_hub, dx_hub))
        no_vel_yaw_robot_deg = self._wrap_deg(no_vel_yaw_field_deg - heading_deg)
        yaw_delta_deg = self._wrap_deg(command["turret_yaw_deg"] - no_vel_yaw_robot_deg)

        arrow_len = 0.95
        comp_rad = np.radians(compensated_yaw_field_deg)
        novel_rad = np.radians(no_vel_yaw_field_deg)

        self.ax.arrow(
            launch_xy[0],
            launch_xy[1],
            arrow_len * np.cos(comp_rad),
            arrow_len * np.sin(comp_rad),
            width=0.012,
            length_includes_head=True,
            head_width=0.10,
            head_length=0.14,
            color=color_comp,
            alpha=0.9,
            zorder=12,
        )
        self.ax.arrow(
            launch_xy[0],
            launch_xy[1],
            arrow_len * np.cos(novel_rad),
            arrow_len * np.sin(novel_rad),
            width=0.006,
            length_includes_head=True,
            head_width=0.08,
            head_length=0.12,
            color=color_novel,
            alpha=0.85,
            linestyle="--",
            zorder=12,
        )
        return no_vel_yaw_robot_deg, yaw_delta_deg

    def _draw_single(self, data):
        self._clear_plot_objects()
        hub_center = self._draw_common_hub(data)

        if self.point_a is not None:
            self.ax.scatter([self.point_a[0]], [self.point_a[1]], c="black", s=50, zorder=11, label="Robot Start")
        if self.point_a is not None and self.point_b is not None:
            dx = self.point_b[0] - self.point_a[0]
            dy = self.point_b[1] - self.point_a[1]
            self.ax.arrow(
                self.point_a[0],
                self.point_a[1],
                dx,
                dy,
                width=0.02,
                length_includes_head=True,
                head_width=0.16,
                head_length=0.22,
                color="#222222",
                alpha=0.9,
                zorder=11,
            )

        traj = data["trajectory"]
        if len(traj) >= 2:
            xy = np.array([[p["x_m"], p["y_m"]] for p in traj], dtype=float)
            z = np.array([p["z_m"] for p in traj], dtype=float)
            points = xy.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            segment_colors = 0.5 * (z[:-1] + z[1:])
            line = plt.matplotlib.collections.LineCollection(segments, cmap="turbo", linewidths=2.5)
            line.set_array(segment_colors)
            self.ax.add_collection(line)
            self.cbar = self.fig.colorbar(line, ax=self.ax, fraction=0.046, pad=0.04)
            self.cbar.set_label("Ball Height z [m]")
            self.ax.scatter([xy[0, 0]], [xy[0, 1]], c="green", s=40, zorder=12, label="Launch")
            self.ax.scatter([xy[-1, 0]], [xy[-1, 1]], c="red", s=40, zorder=12, label="End")

        command = data["command"]
        result = data["result"]
        launch_xy = (traj[0]["x_m"], traj[0]["y_m"]) if len(traj) >= 1 else self.point_a
        no_vel_yaw, yaw_delta = np.nan, np.nan
        if launch_xy is not None:
            no_vel_yaw, yaw_delta = self._draw_turret_arrows(
                launch_xy, hub_center, self.heading_deg, command, "#d62728", "#1f77b4"
            )

        info = (
            f"mode=single | hit={result['hit']} | solve={command['solve_status']}\n"
            f"closest={result['closest_distance_m']:.3f} m | flight={result['flight_time_s']:.3f} s\n"
            f"yaw={command['turret_yaw_deg']:.2f} deg | yaw_no_vel={no_vel_yaw:.2f} deg | "
            f"yaw_delta={yaw_delta:.2f} deg\n"
            f"top={command['top_rpm']:.1f} rpm | bottom={command['bottom_rpm']:.1f} rpm\n"
            f"robot_heading={self.heading_deg:.2f} deg | robot_speed={self.robot_speed_mps:.3f} m/s"
        )
        self._draw_status(info)
        self.ax.legend(loc="lower right")
        self.fig.canvas.draw_idle()

    def _draw_curve(self, shots_data, curve_points):
        self._clear_plot_objects()
        hub_center = self._draw_common_hub(shots_data[0])

        p0, p1, p2 = curve_points
        self.ax.plot(
            [p0[0], p1[0], p2[0]],
            [p0[1], p1[1], p2[1]],
            linestyle=":",
            linewidth=1.0,
            color="#888888",
            alpha=0.8,
            zorder=8,
        )
        dense_t = np.linspace(0.0, 1.0, 80)
        curve_xy = np.array([self._quadratic_bezier(p0, p1, p2, t) for t in dense_t])
        self.ax.plot(curve_xy[:, 0], curve_xy[:, 1], linestyle="--", linewidth=2.0, color="#555555", zorder=8, label="Path")
        self.ax.scatter([p0[0]], [p0[1]], c="black", s=45, zorder=11, label="Path Start")
        self.ax.scatter([p2[0]], [p2[1]], c="black", s=45, zorder=11, label="Path End")

        cmap = plt.get_cmap("tab10")
        hits = 0
        closest_errors = []
        yaw_deltas = []

        for i, shot in enumerate(shots_data):
            color = cmap(i % 10)
            traj = shot["trajectory"]
            command = shot["command"]
            result = shot["result"]
            if result["hit"]:
                hits += 1
            closest_errors.append(float(result["closest_distance_m"]))

            if len(traj) >= 2:
                xy = np.array([[p["x_m"], p["y_m"]] for p in traj], dtype=float)
                self.ax.plot(xy[:, 0], xy[:, 1], color=color, linewidth=2.2, alpha=0.9, zorder=9, label=f"Shot {i + 1}")
                launch_xy = (xy[0, 0], xy[0, 1])
                end_xy = (xy[-1, 0], xy[-1, 1])
                self.ax.scatter([launch_xy[0]], [launch_xy[1]], c=[color], s=25, zorder=12)
                self.ax.scatter([end_xy[0]], [end_xy[1]], c=[color], s=25, marker="x", zorder=12)
            else:
                launch_xy = (shot["input"]["x_m"], shot["input"]["y_m"])

            heading_deg = shot["input"]["heading_deg"]
            _, yaw_delta = self._draw_turret_arrows(
                launch_xy, hub_center, heading_deg, command, color, "#1f77b4"
            )
            yaw_deltas.append(yaw_delta)

        mean_closest = float(np.mean(closest_errors)) if closest_errors else float("nan")
        mean_yaw_delta = float(np.nanmean(yaw_deltas)) if yaw_deltas else float("nan")
        valid_count = len(shots_data)
        side_name = "left" if self.curve_side_sign > 0 else "right"

        info = (
            f"mode=curve | shots={valid_count} | hits={hits}/{valid_count}\n"
            f"mean_closest={mean_closest:.3f} m | mean_yaw_delta={mean_yaw_delta:.2f} deg\n"
            f"robot_speed={self.robot_speed_mps:.3f} m/s | curve_side={side_name}\n"
            "turret arrows: colored=compensated, blue=no-velocity reference"
        )
        self._draw_status(info)
        self.ax.legend(loc="lower right", ncol=2, fontsize=8)
        self.fig.canvas.draw_idle()

    def _compute_single_and_draw(self):
        if self.point_a is None or self.point_b is None:
            return
        dx = self.point_b[0] - self.point_a[0]
        dy = self.point_b[1] - self.point_a[1]
        vx, vy = self._compute_velocity_from_direction(dx, dy, self.robot_speed_mps)
        data = self._run_single_scenario(
            x=self.point_a[0],
            y=self.point_a[1],
            heading_deg=self.heading_deg,
            vx=vx,
            vy=vy,
        )
        self._draw_single(data)

    def _compute_curve_and_draw(self):
        if self.point_a is None or self.point_b is None:
            return
        p0 = np.array(self.point_a, dtype=float)
        p2 = np.array(self.point_b, dtype=float)
        p1 = self._compute_curve_control(p0, p2)

        shots_data = []
        for t in np.linspace(0.10, 0.90, self.curve_shot_count):
            pos = self._quadratic_bezier(p0, p1, p2, t)
            tangent = self._quadratic_bezier_tangent(p0, p1, p2, t)
            tx, ty = float(tangent[0]), float(tangent[1])
            vx, vy = self._compute_velocity_from_direction(tx, ty, self.robot_speed_mps)
            heading_deg = np.degrees(np.arctan2(ty, tx)) if np.hypot(tx, ty) > 1e-9 else 0.0
            shot = self._run_single_scenario(
                x=float(pos[0]),
                y=float(pos[1]),
                heading_deg=float(heading_deg),
                vx=float(vx),
                vy=float(vy),
            )
            shots_data.append(shot)

        self._draw_curve(shots_data, (p0, p1, p2))

    def _compute_and_draw(self):
        if self.point_a is None or self.point_b is None:
            return
        try:
            if self.mode == "single":
                self._compute_single_and_draw()
            else:
                self._compute_curve_and_draw()
        except RuntimeError as exc:
            self._clear_plot_objects()
            self._draw_status(f"Runner error:\n{exc}")
            self.fig.canvas.draw_idle()

    def on_click(self, event):
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        x, y = float(event.xdata), float(event.ydata)
        if self.point_a is None:
            self.point_a = (x, y)
            self.point_b = None
            self._clear_plot_objects()
            self.ax.scatter([x], [y], c="black", s=50, zorder=11)
            self._draw_status("Point A set. Click Point B. " + self._help_text())
            self.fig.canvas.draw_idle()
            return
        if self.point_b is None:
            self.point_b = (x, y)
            self._compute_and_draw()
            return

        self.point_a = (x, y)
        self.point_b = None
        self._clear_plot_objects()
        self.ax.scatter([x], [y], c="black", s=50, zorder=11)
        self._draw_status("Point A reset. Click Point B. " + self._help_text())
        self.fig.canvas.draw_idle()

    def on_key(self, event):
        if event.key == "r":
            self.point_a = None
            self.point_b = None
            self._clear_plot_objects()
            self._draw_status(self._help_text())
            self.fig.canvas.draw_idle()
        elif event.key == "m":
            self.mode = "curve" if self.mode == "single" else "single"
            self.point_a = None
            self.point_b = None
            self._clear_plot_objects()
            self._draw_status(self._help_text())
            self.fig.canvas.draw_idle()
        elif event.key == "+":
            self.robot_speed_mps *= 1.10
            if self.point_a is not None and self.point_b is not None:
                self._compute_and_draw()
        elif event.key == "-":
            self.robot_speed_mps *= 0.90
            if self.point_a is not None and self.point_b is not None:
                self._compute_and_draw()
        elif event.key == "a":
            if self.mode == "single":
                self.heading_deg -= self.heading_step_deg
                if self.point_a is not None and self.point_b is not None:
                    self._compute_and_draw()
        elif event.key == "d":
            if self.mode == "single":
                self.heading_deg += self.heading_step_deg
                if self.point_a is not None and self.point_b is not None:
                    self._compute_and_draw()
        elif event.key == "v":
            if self.mode == "curve":
                self.curve_side_sign *= -1.0
                if self.point_a is not None and self.point_b is not None:
                    self._compute_and_draw()
        elif event.key == "q":
            plt.close(self.fig)


def main():
    ui = ScenarioUI()
    plt.show()


if __name__ == "__main__":
    main()
