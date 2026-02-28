package frc.robot.commands.shooter;

import java.awt.Robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootCommand extends Command {
    
    public ShootCommand() {
        // addRequirements(RobotContainer.shooter, RobotContainer.indexer); 
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        Pair<Double, Double> rpm = RobotContainer.calculationSubsystem.getRPMShooter();
        RobotContainer.shooter.run(rpm.getSecond(), rpm.getFirst());
        if (RobotContainer.shooter.isAtSetpoint())
            RobotContainer.indexer.run(Constants.Indexer.defaultRPM);
        else 
            RobotContainer.indexer.stop();
        RobotContainer.feeder.run(Constants.Feeder.defaultRPM);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.shooter.stop();
        RobotContainer.indexer.stop();
        RobotContainer.feeder.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}