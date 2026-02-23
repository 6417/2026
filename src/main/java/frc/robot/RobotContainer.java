package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CalculationSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
    public static final Pigeon2 gyro;
    // public static final IntakeSubsystem intake;
    public static final SwerveSubsystem drive;
    public static final VisionSubsystem vision;
    // public static final ClimberSubsystem climber;
    // public static final TurretSubsystem turret;
    public static final IndexerSubsystem indexer;
    public static final ShooterSubsystem shooter;
    public static final CalculationSubsystem calculationSubsystem;

    public static final Controls controls;

    private static final SendableChooser<String> autoChooser = new SendableChooser<>();

    static {
        // intake = new IntakeSubsystem();
        gyro = new Pigeon2(Constants.Gyro.PIGEON_ID);
        vision = new VisionSubsystem(true, true);
        drive = new SwerveSubsystem();
        // turret = new TurretSubsystem();
        // climber = new ClimberSubsystem();
        shooter = new ShooterSubsystem();
        indexer = new IndexerSubsystem();
        calculationSubsystem = new CalculationSubsystem();
        controls = new Controls();

        autoChooser.addOption("Example Auto", "Example Auto");
        autoChooser.addOption("Second Auto", "Second Auto");
        autoChooser.addOption("None", null);
        autoChooser.setDefaultOption("Example Auto", "Example Auto");
        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public Command getAutonomousCommand() {
        return drive.getAutonomousCommand(autoChooser.getSelected());
    }
}
