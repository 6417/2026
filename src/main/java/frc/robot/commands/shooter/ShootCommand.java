package frc.robot.commands.shooter;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.Controls.DriveSpeed;
import frc.robot.utils.Utils;

public class ShootCommand extends Command {

    public ShootCommand() {
        addRequirements(RobotContainer.shooter, RobotContainer.indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pair<Double, Double> rpm;
        Pair<Double, Double> calculated = RobotContainer.calculationSubsystem.getRPMShooter();

        if (RobotContainer.climber.robotIsClimbed)
            rpm = new Pair<Double, Double>(calculated.getFirst() + Constants.Climber.climberBonus, calculated.getSecond() + Constants.Climber.climberBonus);
        else 
            rpm = calculated;

        RobotContainer.feeder.run(Constants.Feeder.defaultRPM);
        RobotContainer.indexer.run(Constants.Indexer.defaultRPM);

        RobotContainer.shooter.run(rpm.getSecond(), rpm.getFirst());
       

        if (Utils.isRobotNotInAllianceZone()) {
            RobotContainer.controls.setActiveSpeedFactor(DriveSpeed.DEFAULT_SPEED);
            Controls.shootFactor = 0.3;
        } else {
            RobotContainer.controls.setActiveSpeedFactor(DriveSpeed.SLOW);
            Controls.shootFactor = 1;
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.shooter.stop();
        RobotContainer.indexer.stop();
        RobotContainer.feeder.stop();
        RobotContainer.controls.setActiveSpeedFactor(DriveSpeed.DEFAULT_SPEED);
        Controls.shootFactor = 1;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}