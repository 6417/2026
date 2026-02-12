package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
    public static final Pigeon2 gyro;
    public static final Controls controls;
    public static final SwerveSubsystem drive;
    public static final TurretSubsystem turret;
    public static final ShooterSubsystem shooter;

    private static final SendableChooser<String> autoChooser = new SendableChooser<>();

    static {
        gyro = new Pigeon2(Constants.Gyro.PIGEON_ID);
        drive = new SwerveSubsystem();
        turret = new TurretSubsystem();
        shooter = new ShooterSubsystem();
        controls = new Controls();

        autoChooser.addOption("Example Auto", "Example Auto");
        autoChooser.addOption("Second Auto", "Second Auto");
        autoChooser.addOption("None", null);
        autoChooser.setDefaultOption("None", null);
        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public Command getAutonomousCommand() {
        return drive.getAutonomousCommand(autoChooser.getSelected());
    }
}
