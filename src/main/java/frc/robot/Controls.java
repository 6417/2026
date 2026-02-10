package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.DriveToShootpos;
import frc.robot.commands.drive.DriveToTrench;
import frc.robot.commands.turret.TurretControlled;

/**
 * Holds the data concerning input, which should be available
 * either to the entire program or get exported to the shuffleboard
 */
public class Controls implements Sendable {
    public CommandXboxController driveJoystick = new CommandXboxController(Constants.Joystick.driveJoystickId);
    public CommandXboxController operatorJoystick = new CommandXboxController(Constants.Joystick.operatorJoystickId);

    Trigger ltButtonDrive = driveJoystick.leftTrigger();
    Trigger rtButtonDrive = driveJoystick.rightTrigger();
    Trigger lbButtonDrive = driveJoystick.leftBumper();
    Trigger rbButtonDrive = driveJoystick.rightBumper();
    Trigger aButtonDrive = driveJoystick.a();
    Trigger bButtonDrive = driveJoystick.b();
    Trigger xButtonDrive = driveJoystick.x();
    Trigger yButtonDrive = driveJoystick.y();
    Trigger windowsButtonDrive = driveJoystick.back();
    Trigger burgerButtonDrive = driveJoystick.start();
    Trigger pov0Drive = driveJoystick.povUp();
    Trigger speedKnopf = driveJoystick.leftStick();

    Trigger ltButtonOperator = operatorJoystick.leftTrigger();
    Trigger rtButtonOperator = operatorJoystick.rightTrigger();
    Trigger lbButtonOperator = operatorJoystick.leftBumper();
    Trigger rbButtonOperator = operatorJoystick.rightBumper();
    Trigger aButtonOperator = operatorJoystick.a();
    Trigger bButtonOperator = operatorJoystick.b();
    Trigger xButtonOperator = operatorJoystick.x();
    Trigger yButtonOperator = operatorJoystick.y();
    Trigger windowsButtonOperator = operatorJoystick.back();
    Trigger burgerButtonOperator = operatorJoystick.start();
    Trigger pov0Operator = operatorJoystick.povUp();

    private boolean automatedTurret = true;

    public enum DriveSpeed {
        DEFAULT_SPEED,
        FAST,
        SLOW
    }

    public enum DriveOrientation {
        FieldOriented, Forwards, Backwards
    }

    public Map<DriveSpeed, Double> speedFactors = Map.of(
            DriveSpeed.DEFAULT_SPEED, 1.0,
            DriveSpeed.FAST, 0.9,
            DriveSpeed.SLOW, 0.3);
    private DriveSpeed activeSpeedFactor = DriveSpeed.DEFAULT_SPEED;
    private double accelerationSensitivity = speedFactors.get(activeSpeedFactor);

    public static double deadBandDrive = 0.08;
    public static double deadBandTurn = 0.08;
    public boolean inputsSquared = false;

    public boolean slewRateLimited = true;
    public double slewRateLimit = 1.0;

    public double turnSensitivity = 0.08;

    public DriveOrientation driveOrientation = DriveOrientation.Forwards;

    public void setActiveSpeedFactor(DriveSpeed speedFactor) {
        activeSpeedFactor = speedFactor;
        accelerationSensitivity = speedFactors.get(speedFactor);
    }

    public DriveSpeed getActiveSpeedFactor() {
        return activeSpeedFactor;
    }

    public double getAccelerationSensitivity() {
        return accelerationSensitivity;
    }

    public boolean isTurretAutomated() {
        return automatedTurret;
    }

    public Controls() {
        speedKnopf.whileTrue(Commands.startEnd(
                () -> {
                    setActiveSpeedFactor(DriveSpeed.SLOW);
                },
                () -> {
                    setActiveSpeedFactor(DriveSpeed.DEFAULT_SPEED);
                }));

        burgerButtonDrive.onTrue(new InstantCommand(() -> {
            RobotContainer.drive.zeroGyroWithAlliance();
        }));
        rbButtonDrive.whileTrue(new DriveToTrench(RobotContainer.drive));
        rtButtonDrive.debounce(0.02).whileTrue(new InstantCommand( () -> RobotContainer.drive.setIntakeMode(true)))
        .onFalse(new InstantCommand( () -> RobotContainer.drive.setIntakeMode(false)));

        lbButtonDrive.whileTrue(new DriveToShootpos(RobotContainer.drive, RobotContainer.turret));

        xButtonDrive.onTrue(new InstantCommand(()-> RobotContainer.drive.lock()));
        yButtonOperator.onTrue(new SequentialCommandGroup(new InstantCommand(() -> automatedTurret = !automatedTurret), new TurretControlled(RobotContainer.turret)));

        Shuffleboard.getTab("Drive").add("Controls", this);
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

    // Shuffleboard
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");

        builder.addDoubleProperty("turnSensitivity", () -> turnSensitivity,
                val -> turnSensitivity = val);

        builder.addDoubleProperty("defaultSpeedFactor", () -> speedFactors.get(DriveSpeed.DEFAULT_SPEED),
                val -> speedFactors.put(DriveSpeed.DEFAULT_SPEED, val));
        builder.addDoubleProperty("slowSpeedFactor", () -> speedFactors.get(DriveSpeed.SLOW),
                val -> speedFactors.put(DriveSpeed.SLOW, val));
        builder.addDoubleProperty("fastSpeedFactor", () -> speedFactors.get(DriveSpeed.FAST),
                val -> speedFactors.put(DriveSpeed.FAST, val));
        builder.addDoubleProperty("Current Speed Factor", () -> accelerationSensitivity, null);
        builder.addBooleanProperty("SlewRateLimiter", () -> slewRateLimited,
                val -> slewRateLimited = val);
        builder.addDoubleProperty("SlewRate Limit", () -> slewRateLimit, null);
        builder.addBooleanProperty("SquareInputs", () -> inputsSquared, val -> inputsSquared = val);
    }

}