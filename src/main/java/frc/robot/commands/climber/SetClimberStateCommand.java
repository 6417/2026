package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;

public class SetClimberStateCommand extends Command {
    private final ClimberState targetState;

    public SetClimberStateCommand(ClimberState targetState) {
        // Stateless command: just set the target state and finish.
        this.targetState = targetState;
        // addRequirements(RobotContainer.climber);
    }

    @Override
    public void initialize() {
        // Triggers the Motion Magic move inside the subsystem.
        RobotContainer.climber.setTargetState(targetState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
