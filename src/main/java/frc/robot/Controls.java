package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Holds the data concerning input, which should be available
 * either to the entire program or get exported to the shuffleboard
 */
public class Controls implements Sendable {
    public CommandXboxController driveJoystick = new CommandXboxController(Constants.Joystick.driveJoystickId);

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
            DriveSpeed.SLOW, 0.1);
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

    public Controls() {
        // rbButtonDrive.whileTrue(new ChaseTagCommand(RobotContainer.drive,
        // Constants.OffsetsToAprilTags.offsetToAprilTagRight));
        // lbButtonDrive.whileTrue(new ChaseTagCommand(RobotContainer.drive,
        // Constants.OffsetsToAprilTags.offsetToAprilTagLeft));
        
        rtButtonDrive.whileTrue(Commands.startEnd(
            () -> {
                    setActiveSpeedFactor(DriveSpeed.SLOW);
                },
                () -> {
                    setActiveSpeedFactor(DriveSpeed.DEFAULT_SPEED);
                }));
                
                windowsButtonDrive.onTrue(new InstantCommand(() -> RobotContainer.gyro.reset())); 
        burgerButtonDrive.onTrue(new InstantCommand(()->{
            RobotContainer.drive.zeroGyro();
        }));
        Shuffleboard.getTab("Drive").add("Controls", this);
    }

    public double[] getJoystickAxes(){
        double[] joystickAxes = {
            driveJoystick.getLeftX(),
            driveJoystick.getLeftY(),
            driveJoystick.getRightX(),
            driveJoystick.getRightY()
        };
        for (int i = 0; i < joystickAxes.length; i++) {
            if (Math.abs(joystickAxes[i]) < Controls.deadBandDrive) {
                joystickAxes[i] = 0.0;
            }
            joystickAxes[i]*= getAccelerationSensitivity();
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