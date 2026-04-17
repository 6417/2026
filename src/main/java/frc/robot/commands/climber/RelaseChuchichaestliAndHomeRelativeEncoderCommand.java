package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RelaseChuchichaestliAndHomeRelativeEncoderCommand extends SequentialCommandGroup {
    public RelaseChuchichaestliAndHomeRelativeEncoderCommand() {
        addRequirements(RobotContainer.climber);
        
        addCommands(
            new ClearHatchetForMovement(),
            new HomingMovement(),
            new WaitCommand(0.5),
            new InstantCommand(()->RobotContainer.climber.stop())
        );
    }
    
    // Inner Command für die eigentliche Homing-Bewegung
    private static class HomingMovement extends Command {
        private boolean done = false;

        public HomingMovement() {
            addRequirements(RobotContainer.climber);
        }

        @Override
        public void initialize() {
            done = false;
            RobotContainer.climber.startHoming();
        }

        @Override
        public void execute() {
            if (RobotContainer.climber.isMotorBlockedDetectionByAmperage(Constants.Climber.homingAmpsThreshold)) {
                done = true;
            }
        }

        @Override
        public void end(boolean interrupted) {
            RobotContainer.climber.endHoming();
        }

        @Override
        public boolean isFinished() {
            return done;
        }
    }
}
