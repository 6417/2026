package frc.robot.sim;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import frc.robot.Constants;

/**
 * Exports shot simulation data (RPM sweep + trajectories) to CSV files for plotting.
 */
public class ShotPlotDataExporter {
    private static final int NUM_POINTS = 50;
    private static final double RPM_MIN = 2500.0;
    private static final double RPM_MAX = 6000.0;
    private static final double DT_SEC = 0.005;

    private record Sample(double tSec, double xMeters, double zMeters) {
    }

    private record TrajectoryMetrics(
            double distanceAtHubHeightMeters,
            double rangeAtGroundMeters,
            double maxHeightMeters,
            double timeAtHubHeightSec) {
    }

    public static void main(String[] args) throws IOException {
        Path outputDir = Path.of("build", "shot_plots");
        Files.createDirectories(outputDir);

        Path rpmDistanceCsv = outputDir.resolve("rpm_distance.csv");
        Path trajectoriesCsv = outputDir.resolve("trajectories.csv");

        List<String> distanceRows = new ArrayList<>();
        List<String> trajectoryRows = new ArrayList<>();
        distanceRows.add("rpm,distance_at_hub_height_m,range_at_ground_m,max_height_m,time_at_hub_height_s,hub_height_m");
        trajectoryRows.add("rpm,time_s,x_m,z_m");

        for (int i = 0; i < NUM_POINTS; i++) {
            double rpm = RPM_MIN + ((RPM_MAX - RPM_MIN) * i / (NUM_POINTS - 1));
            double muzzleSpeedMps = rpm * Constants.Shooter.rpmToMpsFactor;
            List<Sample> samples = simulateTrajectory(muzzleSpeedMps);
            TrajectoryMetrics metrics = computeMetrics(samples);

            distanceRows.add(String.format(
                    Locale.US,
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                    rpm,
                    metrics.distanceAtHubHeightMeters(),
                    metrics.rangeAtGroundMeters(),
                    metrics.maxHeightMeters(),
                    metrics.timeAtHubHeightSec(),
                    Constants.ShooterSim.hubCenterHeightMeters));

            for (Sample sample : samples) {
                trajectoryRows.add(String.format(
                        Locale.US,
                        "%.6f,%.6f,%.6f,%.6f",
                        rpm,
                        sample.tSec(),
                        sample.xMeters(),
                        sample.zMeters()));
            }
        }

        Files.write(rpmDistanceCsv, distanceRows, StandardCharsets.UTF_8);
        Files.write(trajectoriesCsv, trajectoryRows, StandardCharsets.UTF_8);

        System.out.println("Exported CSV files:");
        System.out.println(" - " + rpmDistanceCsv.toAbsolutePath());
        System.out.println(" - " + trajectoriesCsv.toAbsolutePath());
    }

    private static List<Sample> simulateTrajectory(double muzzleSpeedMps) {
        List<Sample> samples = new ArrayList<>();
        double pitchRad = Constants.ShooterSim.launchPitchRad;
        double g = Constants.ShooterSim.gravityMetersPerSec2;
        double drag = Constants.ShooterSim.dragCoefficientPerMeter;

        double t = 0.0;
        double x = 0.0;
        double z = Constants.ShooterSim.launchHeightMeters;
        double vx = muzzleSpeedMps * Math.cos(pitchRad);
        double vz = muzzleSpeedMps * Math.sin(pitchRad);

        samples.add(new Sample(t, x, z));
        while (t < Constants.ShooterSim.maxFlightTimeSec && z >= 0.0) {
            double speed = Math.hypot(vx, vz);
            double dragFactor = drag * speed;
            double ax = -dragFactor * vx;
            double az = -dragFactor * vz - g;

            vx += ax * DT_SEC;
            vz += az * DT_SEC;
            x += vx * DT_SEC;
            z += vz * DT_SEC;
            t += DT_SEC;

            samples.add(new Sample(t, x, z));
        }
        return samples;
    }

    private static TrajectoryMetrics computeMetrics(List<Sample> samples) {
        double hubHeight = Constants.ShooterSim.hubCenterHeightMeters;
        double maxHeight = 0.0;
        int peakIndex = 0;
        for (int i = 0; i < samples.size(); i++) {
            if (samples.get(i).zMeters() > maxHeight) {
                maxHeight = samples.get(i).zMeters();
                peakIndex = i;
            }
        }

        double distanceAtHubHeight = 0.0;
        double timeAtHubHeight = 0.0;
        if (maxHeight >= hubHeight) {
            double minError = Double.POSITIVE_INFINITY;
            for (int i = peakIndex; i < samples.size(); i++) {
                double err = Math.abs(samples.get(i).zMeters() - hubHeight);
                if (err < minError) {
                    minError = err;
                    distanceAtHubHeight = samples.get(i).xMeters();
                    timeAtHubHeight = samples.get(i).tSec();
                }
            }
        }

        double rangeAtGround = samples.get(samples.size() - 1).xMeters();
        for (int i = 1; i < samples.size(); i++) {
            Sample prev = samples.get(i - 1);
            Sample cur = samples.get(i);
            if (prev.zMeters() >= 0.0 && cur.zMeters() < 0.0) {
                double alpha = prev.zMeters() / (prev.zMeters() - cur.zMeters());
                rangeAtGround = prev.xMeters() + alpha * (cur.xMeters() - prev.xMeters());
                break;
            }
        }

        return new TrajectoryMetrics(distanceAtHubHeight, rangeAtGround, maxHeight, timeAtHubHeight);
    }
}
