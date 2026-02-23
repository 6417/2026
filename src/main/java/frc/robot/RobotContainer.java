package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class RobotContainer {
    public static final Controls controls;
    public static final IndexerSubsystem indexer;
    static {
        controls = new Controls();
        indexer = new IndexerSubsystem();
    }

}
