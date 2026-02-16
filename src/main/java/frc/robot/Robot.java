package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.ShooterField2dVisualizer;

public class Robot extends LoggedRobot {
    private final RobotContainer robotContainer;
    private Command autonomousCommand;
    private ShooterField2dVisualizer shooterField2dVisualizer;
    private double nextAutoDebugShotTimestampSec = Double.POSITIVE_INFINITY;

    public Robot() {
        Logger.recordMetadata("ProjectName", "MergedShooterTurretProject");

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            String replayPath = System.getenv("AKIT_LOG_PATH");
            if (replayPath != null && !replayPath.isBlank()) {
                setUseTiming(false);
                Logger.setReplaySource(new WPILOGReader(replayPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(replayPath, "_sim")));
            } else {
                // Normal desktop simulation mode (without replay file).
                Logger.addDataReceiver(new NT4Publisher());
                Logger.addDataReceiver(new WPILOGWriter());
            }
        }

        Logger.start();
        LimelightHelpers.SetIMUMode(Constants.Limelight.driveLimelight, 0);
        LimelightHelpers.SetIMUAssistAlpha(Constants.Limelight.driveLimelight, 0.001);
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        RobotContainer.realityCalibrator.periodic();
    }

    @Override
    public void autonomousInit() {
        updateAllianceFieldConstants();

        LimelightHelpers.SetThrottle(Constants.Limelight.driveLimelight, 0);
        RobotContainer.drive.setAutomatedControl();
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (Constants.Field.EDGE == null || Constants.Field.HUB_CENTER == null || Constants.Field.neutralZoneStartX == 0) {
            updateAllianceFieldConstants();
        }

        LimelightHelpers.SetThrottle(Constants.Limelight.driveLimelight, 0);
        LimelightHelpers.SetRobotOrientation(
                Constants.Limelight.driveLimelight,
                RobotContainer.gyro.getYaw().getValueAsDouble(),
                0,
                0,
                0,
                0,
                0);
        RobotContainer.drive.setOpeatorControl();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
        LimelightHelpers.SetThrottle(Constants.Limelight.driveLimelight, 200);
    }

    @Override
    public void disabledPeriodic() {
        LimelightHelpers.SetIMUMode(Constants.Limelight.driveLimelight, 1);
    }

    @Override
    public void testInit() {
        LimelightHelpers.SetThrottle(Constants.Limelight.driveLimelight, 0);
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
        updateAllianceFieldConstants();
        shooterField2dVisualizer = new ShooterField2dVisualizer(
                Constants.Shooter.hubPositionField,
                Constants.ShooterSim.field2dMaxTraceCount,
                Constants.ShooterSim.field2dMaxPointsPerTrace);
        if (Constants.ShooterSim.enableAutoDebugShotsInSim) {
            nextAutoDebugShotTimestampSec = Timer.getFPGATimestamp() + Constants.ShooterSim.autoDebugShotIntervalSec;
        } else {
            nextAutoDebugShotTimestampSec = Double.POSITIVE_INFINITY;
        }
    }

    @Override
    public void simulationPeriodic() {
        Pose2d pose = RobotContainer.drive.getPose();
        ChassisSpeeds fieldVelocity = RobotContainer.drive.getFieldVelocity();
        RobotContainer.shooter.setSimRobotState(
                pose,
                new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond));

        if (Constants.ShooterSim.enableAutoDebugShotsInSim) {
            RobotContainer.shooter.applyShotCommandFromRobotState(
                    pose,
                    new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond));
        }

        if (Constants.ShooterSim.enableAutoDebugShotsInSim
                && Timer.getFPGATimestamp() >= nextAutoDebugShotTimestampSec) {
            boolean fired;
            if (Constants.ShooterSim.autoDebugShotsRequireReady) {
                fired = RobotContainer.shooter.tryFire(
                        RobotContainer.turret.isAtSetpoint(),
                        fieldVelocity.omegaRadiansPerSecond);
            } else {
                RobotContainer.shooter.simulateFireDebug(Constants.ShooterSim.manualTestMuzzleSpeedMps);
                fired = true;
            }
            if (fired) {
                nextAutoDebugShotTimestampSec += Constants.ShooterSim.autoDebugShotIntervalSec;
            }
        }

        if (shooterField2dVisualizer != null) {
            shooterField2dVisualizer.update(
                    pose,
                    RobotContainer.shooter.getCurrentMuzzlePositionField(),
                    RobotContainer.shooter.getCurrentTurretYawField(),
                    RobotContainer.shooter.getLatestActiveShotSample(),
                    RobotContainer.shooter.drainCompletedShotTraces());
        }
    }

    private void updateAllianceFieldConstants() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        boolean blueAlliance = alliance == Alliance.Blue;

        Constants.Field.EDGE = blueAlliance
                ? new Pose2d(0, 0, null)
                : new Pose2d(Constants.Field.FIELD_LENGTH_METERS, Constants.Field.FIELD_WIDTH_METERS, null);

        Constants.Field.HUB_CENTER = blueAlliance
                ? Constants.Field.HUB_CENTER_BLUE
                : Constants.Field.HUB_CENTER_RED;
        Constants.Shooter.hubPositionField = Constants.Field.HUB_CENTER.getTranslation();

        Constants.Field.neutralZoneStartX = blueAlliance
                ? Units.inchesToMeters(Constants.Field.HUB_CENTER_FROM_ALLIANCE_WALL_INCHES)
                : Units.inchesToMeters(
                        Constants.Field.FIELD_LENGTH_INCHES - Constants.Field.HUB_CENTER_FROM_ALLIANCE_WALL_INCHES);
    }
}
