package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    public static final IntakeSubsystem intake;
    public static final Controls controls;
    public static final Pigeon2 gyro;


    static {
        intake = new IntakeSubsystem();
        controls = new Controls();
        gyro = new Pigeon2(Constants.Gyro.PIGEON_ID);

    }
}
