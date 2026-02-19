// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.diagnostics.ClimberDiagnostic;
import frc.robot.diagnostics.DiagnosticReport;
import frc.robot.diagnostics.IntakeDiagnostic;
import frc.robot.diagnostics.SwerveDriveDiagnostic;
import frc.robot.diagnostics.SwerveSteerDiagnostic;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot { // LoggedRobot for AdvantageKit
  private final RobotContainer robotContainer;
  private Command autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Advantage Kit initialization
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
    LimelightHelpers.SetIMUMode(Constants.Limelight.driveLimelight,0);
    LimelightHelpers.SetIMUAssistAlpha(Constants.Limelight.driveLimelight, 0.001);
    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    Constants.Field.EDGE = DriverStation.getAlliance().get() == Alliance.Blue ? 
        new Pose2d(0, 0, null) : 
        new Pose2d(Constants.Field.FIELD_LENGTH_METERS, Constants.Field.FIELD_WIDTH_METERS, null);
    
    Constants.Field.HUB_CENTER = 
        DriverStation.getAlliance().get() == Alliance.Blue ? 
        Constants.Field.HUB_CENTER_BLUE : 
        Constants.Field.HUB_CENTER_RED;

    Constants.Field.neutralZoneStartX = DriverStation.getAlliance().get() == Alliance.Blue ? Units.inchesToMeters(158.6) : Units.inchesToMeters(Constants.Field.FIELD_LENGTH_INCHES -158.6);

    LimelightHelpers.SetThrottle(Constants.Limelight.driveLimelight, 0); // "Enable" Limelight
    RobotContainer.drive.setAutomatedControl();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (Constants.Field.EDGE == null || Constants.Field.HUB_CENTER == null || Constants.Field.neutralZoneStartX == 0) {
      Constants.Field.EDGE = DriverStation.getAlliance().get() == Alliance.Blue ? 
          new Pose2d(0, 0, null) : 
          new Pose2d(Constants.Field.FIELD_LENGTH_METERS, Constants.Field.FIELD_WIDTH_METERS, null);
      
      Constants.Field.HUB_CENTER = 
          DriverStation.getAlliance().get() == Alliance.Blue ? 
          Constants.Field.HUB_CENTER_BLUE : 
          Constants.Field.HUB_CENTER_RED;

      Constants.Field.neutralZoneStartX = DriverStation.getAlliance().get() == Alliance.Blue ? Units.inchesToMeters(158.6) : Units.inchesToMeters(Constants.Field.FIELD_LENGTH_INCHES -158.6);
    }

    LimelightHelpers.SetThrottle(Constants.Limelight.driveLimelight, 0); // "Enable" Limelight
    LimelightHelpers.SetRobotOrientation(Constants.Limelight.driveLimelight, RobotContainer.drive.getHeading().getDegrees(), 0, 0, 0, 0, 0); // Seed Limelights IMU with Pigeon 2 yaw
    RobotContainer.drive.setOpeatorControl();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    LimelightHelpers.SetThrottle(Constants.Limelight.driveLimelight, 200); // Sort of disable Limelight, so there is less thermal production
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    LimelightHelpers.SetIMUMode(Constants.Limelight.driveLimelight, 1); //Seed IMU when disabled
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    LimelightHelpers.SetThrottle(Constants.Limelight.driveLimelight, 0); // "Enable" Limelight
    CommandScheduler.getInstance().cancelAll();

    DiagnosticReport report = new DiagnosticReport();
    List<Command> diagnosticChecks = new ArrayList<>();

    if (Constants.Diagnostics.enableSwerveDriveTest) {
      diagnosticChecks.add(new SwerveDriveDiagnostic(RobotContainer.drive, report));
    }
    if (Constants.Diagnostics.enableSwerveSteerTest) {
      diagnosticChecks.add(new SwerveSteerDiagnostic(RobotContainer.drive, report));
    }
    if (Constants.Diagnostics.enableIntakeTest) {
      diagnosticChecks.add(new IntakeDiagnostic(RobotContainer.intake, report));
    }
    if (Constants.Diagnostics.enableClimberTest) {
      diagnosticChecks.add(new ClimberDiagnostic(RobotContainer.climber, report));
    }

    if (!diagnosticChecks.isEmpty()) {
      CommandScheduler.getInstance().schedule(
          new SequentialCommandGroup(diagnosticChecks.toArray(new Command[0])));
    } else {
      Logger.recordOutput("Diagnostics/Overall", "SKIPPED");
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
