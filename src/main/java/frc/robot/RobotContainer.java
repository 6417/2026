package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    public static final ShooterSubsystem shooter;
    public static final IndexerSubsystem indexer;
    public static final Controls controls;

    static {
        shooter = new ShooterSubsystem();
        indexer = new IndexerSubsystem();
        controls = new Controls();
    }

}
