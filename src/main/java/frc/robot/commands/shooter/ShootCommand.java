package frc.robot.commands.shooter;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class ShootCommand extends Command {
    
    public ShootCommand() {
        addRequirements(RobotContainer.shooter, RobotContainer.indexer); 
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        Pair<Double, Double> rpm = RobotContainer.calculationSubsystem.getRPMShooter();
        RobotContainer.shooter.run(rpm.getSecond(), rpm.getFirst());
        if (RobotContainer.shooter.isAtSetpoint() && RobotContainer.turret.isAtSetpoint())
            RobotContainer.indexer.run(Constants.Indexer.defaultRPM);
        else 
            RobotContainer.indexer.stop();
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