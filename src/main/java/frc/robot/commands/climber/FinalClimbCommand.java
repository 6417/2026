package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class FinalClimbCommand extends SequentialCommandGroup {

    public FinalClimbCommand() {
        addRequirements(RobotContainer.climber);
        
        addCommands(
            new ClearHatchetForMovement(),
            new FinalClimbMovement()
        );
    }
    
    // Inner Command für die eigentliche Climb-Bewegung
    private static class FinalClimbMovement extends Command {
        boolean robotIsClimbed = false;

        public FinalClimbMovement() {
            addRequirements(RobotContainer.climber);
        }

        @Override
        public void initialize() {
            robotIsClimbed = false;
            RobotContainer.climber.setManualPercent(Constants.Climber.climbSpeed);
        }

        @Override
        public void execute() {
            if (RobotContainer.climber.isClimberAtPosition(Constants.Climber.climbedPosition)) {
                robotIsClimbed = true;
            }
        }

        @Override
        public void end(boolean interrupted) {
            RobotContainer.climber.enableServoHatchet();
            RobotContainer.climber.stop();
        }

        @Override
        public boolean isFinished() {
            return robotIsClimbed;
        }
    }
}
