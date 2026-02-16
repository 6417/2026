package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.calibration.CancelCalibrationCommand;
import frc.robot.commands.calibration.CommitCalibrationCommand;
import frc.robot.commands.DriveToTrench;
import frc.robot.commands.TurretControlled;
import frc.robot.commands.sim.SimMovingShotScenarioCommand;

/**
 * Combined controls for swerve/turret and shooter simulation.
 */
public class Controls {
    private final CommandXboxController driveJoystick = new CommandXboxController(Constants.Joystick.driveJoystickId);
    private final CommandXboxController operatorJoystick = new CommandXboxController(Constants.Joystick.operatorJoystickId);

    private final Trigger ltButtonDrive = driveJoystick.leftTrigger();
    private final Trigger rtButtonDrive = driveJoystick.rightTrigger();
    private final Trigger rbButtonDrive = driveJoystick.rightBumper();
    private final Trigger burgerButtonDrive = driveJoystick.start();
    private final Trigger aButtonDrive = new Trigger(() -> driveJoystick.getHID().getRawButton(1));

    private final Trigger aButtonOperator = new Trigger(() -> operatorJoystick.getHID().getRawButton(1));
    private final Trigger bButtonOperator = operatorJoystick.b();
    private final Trigger xButtonOperator = operatorJoystick.x();
    private final Trigger yButtonOperator = operatorJoystick.y();
    private final Trigger lbButtonOperator = operatorJoystick.leftBumper();
    private final Trigger rbButtonOperator = operatorJoystick.rightBumper();
    private final Trigger backButtonOperator = operatorJoystick.back();
    private final Trigger startButtonOperator = operatorJoystick.start();
    private final Trigger rightStickButtonOperator = operatorJoystick.rightStick();
    private final Trigger dpadUpOperator = new Trigger(() -> operatorJoystick.getHID().getPOV() == 0);
    private final Trigger dpadRightOperator = new Trigger(() -> operatorJoystick.getHID().getPOV() == 90);
    private final Trigger dpadDownOperator = new Trigger(() -> operatorJoystick.getHID().getPOV() == 180);
    private final Trigger dpadLeftOperator = new Trigger(() -> operatorJoystick.getHID().getPOV() == 270);
    private final Trigger calibrationToggleOperator = backButtonOperator.and(startButtonOperator);

    private boolean automatedTurret = true;

    public enum DriveSpeed {
        DEFAULT_SPEED,
        FAST,
        SLOW
    }

    private final java.util.Map<DriveSpeed, Double> speedFactors = java.util.Map.of(
            DriveSpeed.DEFAULT_SPEED, 1.0,
            DriveSpeed.FAST, 0.9,
            DriveSpeed.SLOW, 0.3);

    private DriveSpeed activeSpeedFactor = DriveSpeed.DEFAULT_SPEED;
    private double accelerationSensitivity = speedFactors.get(activeSpeedFactor);

    public Controls() {
        calibrationToggleOperator.onTrue(Commands.runOnce(() -> RobotContainer.realityCalibrator.toggleMode()));

        rtButtonDrive.whileTrue(Commands.startEnd(
                () -> setActiveSpeedFactor(DriveSpeed.SLOW),
                () -> setActiveSpeedFactor(DriveSpeed.DEFAULT_SPEED)));

        burgerButtonDrive.onTrue(new InstantCommand(() -> RobotContainer.drive.zeroGyro()));

        rbButtonDrive.whileTrue(new DriveToTrench(RobotContainer.drive));

        ltButtonDrive
                .whileTrue(new InstantCommand(() -> RobotContainer.drive.setIntakeMode(true)))
                .onFalse(new InstantCommand(() -> RobotContainer.drive.setIntakeMode(false)));

        yButtonOperator.and(() -> !RobotContainer.realityCalibrator.isModeEnabled()).onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> automatedTurret = !automatedTurret),
                new TurretControlled(RobotContainer.turret)));

        // Reality-calibration bindings (active only when calibration mode is enabled).
        yButtonOperator.and(() -> RobotContainer.realityCalibrator.isModeEnabled())
                .onTrue(Commands.runOnce(() -> RobotContainer.realityCalibrator.cyclePreset()));
        aButtonOperator.and(() -> RobotContainer.realityCalibrator.isModeEnabled())
                .onTrue(Commands.runOnce(() -> RobotContainer.realityCalibrator.runNextShot()));
        xButtonOperator.and(() -> RobotContainer.realityCalibrator.isModeEnabled())
                .onTrue(Commands.runOnce(() -> RobotContainer.realityCalibrator.markHit()));
        bButtonOperator.and(() -> RobotContainer.realityCalibrator.isModeEnabled())
                .onTrue(Commands.runOnce(() -> RobotContainer.realityCalibrator.markMissUnknown()));

        dpadLeftOperator.and(() -> RobotContainer.realityCalibrator.isModeEnabled())
                .onTrue(Commands.runOnce(() -> RobotContainer.realityCalibrator.markMissLeft()));
        dpadRightOperator.and(() -> RobotContainer.realityCalibrator.isModeEnabled())
                .onTrue(Commands.runOnce(() -> RobotContainer.realityCalibrator.markMissRight()));
        dpadUpOperator.and(() -> RobotContainer.realityCalibrator.isModeEnabled())
                .onTrue(Commands.runOnce(() -> RobotContainer.realityCalibrator.markMissLong()));
        dpadDownOperator.and(() -> RobotContainer.realityCalibrator.isModeEnabled())
                .onTrue(Commands.runOnce(() -> RobotContainer.realityCalibrator.markMissShort()));

        rbButtonOperator.and(() -> RobotContainer.realityCalibrator.isModeEnabled())
                .onTrue(Commands.runOnce(() -> RobotContainer.realityCalibrator.computeSuggestion()));
        lbButtonOperator.and(() -> RobotContainer.realityCalibrator.isModeEnabled())
                .onTrue(new CommitCalibrationCommand(RobotContainer.realityCalibrator));
        rightStickButtonOperator.and(() -> RobotContainer.realityCalibrator.isModeEnabled())
                .onTrue(new CancelCalibrationCommand(RobotContainer.realityCalibrator));

        // Shooter simulator bindings (only active in desktop sim).
        if (RobotBase.isSimulation()) {
            var manualTestShotCommand = Commands.sequence(
                    new InstantCommand(
                            () -> RobotContainer.shooter.run(
                                    Constants.ShooterSim.manualTestTopRpm,
                                    Constants.ShooterSim.manualTestBottomRpm),
                            RobotContainer.shooter),
                    Commands.waitSeconds(Constants.ShooterSim.manualTestSpinupSec),
                    new InstantCommand(
                            () -> RobotContainer.shooter.simulateFireDebug(Constants.ShooterSim.manualTestMuzzleSpeedMps),
                            RobotContainer.shooter));
            aButtonOperator.and(() -> !RobotContainer.realityCalibrator.isModeEnabled()).onTrue(manualTestShotCommand);
            aButtonDrive.onTrue(manualTestShotCommand);
            bButtonOperator.and(() -> !RobotContainer.realityCalibrator.isModeEnabled())
                    .whileTrue(new SimMovingShotScenarioCommand(RobotContainer.shooter));
            xButtonOperator.and(() -> !RobotContainer.realityCalibrator.isModeEnabled()).onTrue(new InstantCommand(
                    () -> RobotContainer.shooter.setSimRobotState(
                            new Pose2d(
                                    Constants.ShooterSim.scenarioStartPositionField,
                                    new Rotation2d(Constants.ShooterSim.scenarioStartHeadingRad)),
                            Constants.ShooterSim.scenarioVelocityFieldMps),
                    RobotContainer.shooter));
        }

    }

    public boolean isTurretAutomated() {
        return automatedTurret;
    }

    public void setActiveSpeedFactor(DriveSpeed speedFactor) {
        activeSpeedFactor = speedFactor;
        accelerationSensitivity = speedFactors.get(speedFactor);
    }

    public double getAccelerationSensitivity() {
        return accelerationSensitivity;
    }

    public double[] getJoystickAxes() {
        double[] joystickAxes = {
                driveJoystick.getLeftX(),
                driveJoystick.getLeftY(),
                driveJoystick.getRightX(),
                driveJoystick.getRightY()
        };

        for (int i = 0; i < joystickAxes.length; i++) {
            if (Math.abs(joystickAxes[i]) < Constants.Controls.deadBandDrive) {
                joystickAxes[i] = 0.0;
            }
            joystickAxes[i] *= getAccelerationSensitivity();
        }
        return joystickAxes;
    }
}
