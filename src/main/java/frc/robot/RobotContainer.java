package frc.robot;

import java.util.Map;
import java.util.HashMap;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.ShooterParallelCommandGroup;
import frc.robot.commands.turret.TurretZeroCommand;
import frc.robot.subsystems.CalculationSubsystem;

public class RobotContainer {
    public static final Pigeon2 gyro;
    public static final IntakeSubsystem intake;
    public static final SwerveSubsystem drive;
    public static final VisionSubsystem vision;
    // public static final ClimberSubsystem climber;
    public static final TurretSubsystem turret;
    public static final IndexerSubsystem indexer;
    public static final ShooterSubsystem shooter;
    public static final FeederSubsystem feeder;
    public static final CalculationSubsystem calculationSubsystem;

    public static final Controls controls;

    private static final Map<String,Command> namedCommands;
    private static final SendableChooser<String> autoChooser = new SendableChooser<>();

    static {
        intake = new IntakeSubsystem();
        gyro = new Pigeon2(Constants.Gyro.PIGEON_ID);
        vision = new VisionSubsystem(true, true, true);
        drive = new SwerveSubsystem();
        turret = new TurretSubsystem();
        // climber = new ClimberSubsystem();
        shooter = new ShooterSubsystem();
        feeder = new FeederSubsystem();
        indexer = new IndexerSubsystem();
        calculationSubsystem = new CalculationSubsystem();
        controls = new Controls();

        namedCommands = new HashMap<String,Command>();

        namedCommands.put("ZeroTurret", new TurretZeroCommand(turret));
        namedCommands.put("Shoot", new ShooterParallelCommandGroup());
        namedCommands.put("Intake", new IntakeCommand(intake));

        NamedCommands.registerCommands(namedCommands);

        autoChooser.addOption("Links", "Left");
        autoChooser.addOption("Rechts", "Right");
        autoChooser.addOption("None", null);
        autoChooser.setDefaultOption("Example Auto", "Example Auto");
        
        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public Command getAutonomousCommand() {
        return drive.getAutonomousCommand(autoChooser.getSelected());
    }
}
