package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;

public class RelaseChuchichaestliAndHomeRelativeEncoderCommand extends Command {
    private boolean done = false;
    private final ClimberSubsystem climber;
    public RelaseChuchichaestliAndHomeRelativeEncoderCommand(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(this.climber);
    }

    @Override
    public void initialize(){
        done = false;
        climber.startHoming();
    }

    @Override
    public void execute() {
        if (climber.isMotorBlockedDetectionByAmperage(Constants.Climber.homingAmpsThreshold)) {
            climber.endHoming();
            done = true; 
        }

    }

    @Override
    public void end(boolean interrupted) {
        // Stop motor on release or interruption.
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
