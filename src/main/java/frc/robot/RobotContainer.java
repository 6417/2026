package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    public static final SwerveDrive drive;
    public static final Controls controls;
    public static final Pigeon2 gyro;

    public static final SendableChooser<Command> autoChooser;

    static {
        drive = new SwerveDrive();
        controls = new Controls();
        gyro = new Pigeon2(Constants.Gyro.PIGEON_ID);


        //TODO: Autonomous chooser setup -> choreo or pathplanner?
        SmartDashboard.putData("Auto", autoChooser);

    }

    public Command getAutonomousCommand(){
        return autoChooser.getSelected();
    }
}
