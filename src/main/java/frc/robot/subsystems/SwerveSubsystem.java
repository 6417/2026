package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.lang.invoke.ConstantBootstraps;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive drive;
    private VisionSubsystem vision;
    private boolean driveIsAutomated;

    private final boolean blueAlliance;
    private boolean intakeMode;

    public SwerveSubsystem() {
        blueAlliance = getAlliance() == Alliance.Blue;
        vision = new VisionSubsystem(true);
        driveIsAutomated = false;
        intakeMode = false;

        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1), // not very accurate -> gets
                                                                                       // corrected by limelight anyways
                Meter.of(4)),
                Rotation2d.fromDegrees(0))
                : new Pose2d(new Translation2d(Meter.of(16),
                        Meter.of(4)),
                        Rotation2d.fromDegrees(180));

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            drive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(Constants.SwerveSubsystem.maxSpeed, startingPose);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        drive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via
                                           // angle.
        drive.setCosineCompensator(false);// !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
                                          // simulations since it causes discrepancies not seen in real life.
        drive.setAngularVelocityCompensation(true,
                true,
                0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
                      // coefficient of 0.1.
        drive.setModuleEncoderAutoSynchronize(false,
                1); // Enable if you want to resynchronize your absolute encoders and motor encoders
                    // periodically when they are not moving.

        replaceSwerveModuleFeedforward(Constants.SwerveSubsystem.feedforward);

        if (Constants.Limelight.useVision) {
            drive.stopOdometryThread();

        }
        setupPathPlanner();
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
    }

    @Override
    public void periodic() {
        // manually update odometry if using vision

        updateOdometry();
        Logger.recordOutput("Swerve/Odomerty", drive.getPose());

        // TODO: update odometry with vision measurements

        double[] joystickAxes = RobotContainer.controls.getJoystickAxes();
        if (!driveIsAutomated) {
            if (Constants.SwerveSubsystem.oldTurnSystem) {
                if (intakeMode) {
                    driveCommand(
                            () -> -joystickAxes[1],
                            () -> -joystickAxes[0],
                            () -> joystickAxes[0] * 100,
                            () -> joystickAxes[1] * 100).schedule();
                } else {
                    driveCommand(
                            () -> -joystickAxes[1],
                            () -> -joystickAxes[0],
                            () -> -joystickAxes[2]).schedule();
                }
            } else {
                if (intakeMode) {
                    driveCommand(
                            () -> -joystickAxes[1],
                            () -> -joystickAxes[0],
                            () -> -joystickAxes[0],
                            () -> -joystickAxes[1]).schedule();
                } else {
                    int i = DriverStation.getAlliance().get() == Alliance.Blue ? 1 : -1;
                    driveCommand(
                            () -> -joystickAxes[1],
                            () -> -joystickAxes[0],
                            () -> i * -joystickAxes[2],
                            () -> i * -joystickAxes[3]).schedule();
                }
            }
        }
    }

    private void updateOdometry() {
        drive.updateOdometry();
        if (Constants.Limelight.useVision) {
            vision.updateOdometry();
        }
        ;
    };

    public void resetOdometry(Pose2d pose) {
        drive.resetOdometry(pose);
    }

    public void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            final boolean enableFeedforward = false;
            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose,
                    // Robot pose supplier
                    this::resetOdometry,
                    // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotVelocity,
                    // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward) {
                            drive.drive(
                                    speedsRobotRelative,
                                    drive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces());
                        } else {
                            drive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                    // optionally outputs individual module feedforwards
                    new PPHolonomicDriveController(
                            // PPHolonomicController is the built in path following controller for holonomic
                            // drive trains
                            new PIDConstants(5.0, 0.0, 0.0),
                            // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0)
                    // Rotation PID constants
                    ),
                    config,
                    // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this
            // Reference to this subsystem to set requirements
            );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Preload PathPlanner Path finding
        // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
        PathfindingCommand.warmupCommand().schedule();
    }

    public Command getAutonomousCommand(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother
     *                         controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother
     *                         controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for
     *                         smoother controls.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            drive.drive(SwerveMath.scaleTranslation(new Translation2d(
                    translationX.getAsDouble() * drive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * drive.getMaximumChassisVelocity()), 0.8),
                    Math.pow(angularRotationX.getAsDouble(), 1) * drive.getMaximumChassisAngularVelocity(),
                    true,
                    false);
        });
    }

    public Command driveToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                drive.getMaximumChassisVelocity() * 0.6, 4.0,
                drive.getMaximumChassisAngularVelocity() * 0.6, Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
        );
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother
     *                     controls.
     * @param translationY Translation in the Y direction. Cubed for smoother
     *                     controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        // swerveDrive.setHeadingCorrection(true);
        return run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                    translationY.getAsDouble()), 0.8);

            // Make the robot move
            drive.driveFieldOriented(drive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    drive.getOdometryHeading().getRadians(),
                    drive.getMaximumChassisVelocity()));
        });
    }

    /**
     * The primary method for controlling the drivebase. Takes a
     * {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly. Can use either open-loop
     * or closed-loop velocity control for
     * the wheel velocities. Also has field- and robot-relative modes, which affect
     * how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear
     *                      velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards
     *                      the bow (front) and positive y is
     *                      torwards port (left). In field-relative mode, positive x
     *                      is away from the alliance wall
     *                      (field North) and positive y is torwards the left wall
     *                      when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.
     *                      Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode. True for field-relative, false for
     *                      robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        drive.drive(translation,
                rotation,
                fieldRelative,
                false);
    }

    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                        new Config(),
                        this, drive, 12, true),
                3.0, 5.0, 3.0);
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                        new Config(),
                        this, drive),
                3.0, 5.0, 3.0);
    }

    /**
     * Returns a Command that centers the modules of the SwerveDrive subsystem.
     *
     * @return a Command that centers the modules of the SwerveDrive subsystem
     */
    public Command centerModulesCommand() {
        return run(() -> Arrays.asList(drive.getModules())
                .forEach(it -> it.setAngle(0.0)));
    }

    /**
     * Replaces the swerve module feedforward with a new SimpleMotorFeedforward
     * object.
     *
     * @param kS the static gain of the feedforward
     * @param kV the velocity gain of the feedforward
     * @param kA the acceleration gain of the feedforward
     */
    public void replaceSwerveModuleFeedforward(SimpleMotorFeedforward feedforward) {
        drive.replaceSwerveModuleFeedforward(feedforward);
    }

    private Alliance getAlliance() {
        return edu.wpi.first.wpilibj.DriverStation.getAlliance().get();
    }

    public Pose2d getPose() {
        return drive.getPose();
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return drive.kinematics;
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        drive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        drive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroGyro() {
        drive.zeroGyro();
        if (Constants.Limelight.useVision) {
            LimelightHelpers.SetIMUMode(Constants.Limelight.driveLimelight, 1); // Seed IMU when disabled
            LimelightHelpers.SetRobotOrientation(Constants.Limelight.driveLimelight, 0, 0, 0, 0, 0, 0);
            LimelightHelpers.SetIMUMode(Constants.Limelight.driveLimelight, 4);
        }

    }

    public void zeroGyroWithAlliance() {
        if (blueAlliance) {
            drive.zeroGyro();
        } else {
            drive.zeroGyro();
            if (Constants.Limelight.useVision) {
            LimelightHelpers.SetIMUMode(Constants.Limelight.driveLimelight, 1); // Seed IMU when disabled
            LimelightHelpers.SetRobotOrientation(Constants.Limelight.driveLimelight, 180,
            0, 0, 0, 0, 0);
            LimelightHelpers.SetIMUMode(Constants.Limelight.driveLimelight, 4);
            }
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        }
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        drive.setMotorIdleMode(brake);
    }

    public void setAutomatedControl() {
        driveIsAutomated = true;
    }

    public void setOpeatorControl() {
        driveIsAutomated = false;
    }

    public void setIntakeMode(boolean intakeMode) {
        this.intakeMode = intakeMode;
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to
     * resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for
     * speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return drive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                headingX,
                headingY,
                getHeading().getRadians(),
                Constants.SwerveSubsystem.maxSpeed);
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle.
     * Control the robot at an offset of
     * 90deg.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

        return drive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                angle.getRadians(),
                getHeading().getRadians(),
                Constants.SwerveSubsystem.maxSpeed);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return drive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return drive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return drive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return drive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        drive.lockPose();
    }

    /**
     * Gets the swerve drive object.
     *
     * @return {@link SwerveDrive}
     */
    public SwerveDrive getSwerveDrive() {
        return drive;
    }
}
