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
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
    public static final Pigeon2 gyro;
    public static final Controls controls;
    public static final SwerveSubsystem drive;
    public static final TurretSubsystem turret;
    private static final SendableChooser<String> autoChooser = new SendableChooser<>();

    // public static final SendableChooser<Command> autoChooser;

    static {
        gyro = new Pigeon2(Constants.Gyro.PIGEON_ID);
        drive = new SwerveSubsystem();
        turret = new TurretSubsystem();
        controls = new Controls();

        autoChooser.addOption("Example Auto", "Example Auto");
        autoChooser.addOption("Second Auto", "Second Auto");
        autoChooser.addOption("None", null);
        autoChooser.setDefaultOption("Example Auto", "Example Auto");
        SmartDashboard.putData("Auto Selector", autoChooser);

    }

    public Command getAutonomousCommand() {
        // System.out.println("Autonomous Command 'Example Auto' Selected");
        return drive.getAutonomousCommand(autoChooser.getSelected());
        // drive.driveToPose(new Pose2d(1,2, new Rotation2d()));
    }
}
