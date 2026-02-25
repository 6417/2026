package frc.robot.commands.shooter;

import java.awt.Robot;

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
        addRequirements(RobotContainer.shooter, RobotContainer.indexer); 
    }

    @Override
    public void initialize() {
        double rpm = RobotContainer.calculationSubsystem.getRPMShooter();
        RobotContainer.shooter.run(rpm, rpm);
        RobotContainer.indexer.run(1000);
    }
    
    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.shooter.stop();
        RobotContainer.indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}