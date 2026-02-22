package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootCommand extends Command {
    
    public ShootCommand() {
       
        // Command implementation goes here
         
        addRequirements(RobotContainer.shooter); // don't add drive
    }

    @Override
    public void initialize() {
        int rpm = 3000;
        RobotContainer.shooter.run(rpm, rpm);
    }
    
    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}