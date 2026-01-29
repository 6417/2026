package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
    public static final Pigeon2 gyro;
    public static final Controls controls;
    public static final SwerveSubsystem drive;

    // public static final SendableChooser<Command> autoChooser;

    static {
        gyro = new Pigeon2(Constants.Gyro.PIGEON_ID);
        drive = new SwerveSubsystem();
        controls = new Controls();
        



        //TODO: Autonomous chooser setup -> choreo or pathplanner?
        // SmartDashboard.putData("Auto", autoChooser);

    }

    public Command getAutonomousCommand(){
        //System.out.println("Autonomous Command 'Example Auto' Selected");
        return drive.driveToPose(new Pose2d(1,2, new Rotation2d()));//drive.getAutonomousCommand("Example Auto");
    }
}
