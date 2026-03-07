package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;

public class RelaseChuchichaestliAndHomeRelativeEncoderCommand extends SequentialCommandGroup {
    private final ClimberSubsystem climber;

    public RelaseChuchichaestliAndHomeRelativeEncoderCommand(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(this.climber);
        
        addCommands(
            new ClearHatchetForMovement(),
            new HomingMovement(climber),
            new WaitCommand(0.5),
            new InstantCommand(()->RobotContainer.climber.stop())
        );
    }
    
    // Inner Command für die eigentliche Homing-Bewegung
    private static class HomingMovement extends Command {
        private boolean done = false;
        private final ClimberSubsystem climber;

        public HomingMovement(ClimberSubsystem climber) {
            this.climber = climber;
            addRequirements(this.climber);
        }

        @Override
        public void initialize() {
            done = false;
            climber.startHoming();
        }

        @Override
        public void execute() {
            if (climber.isMotorBlockedDetectionByAmperage(Constants.Climber.homingAmpsThreshold)) {
                done = true;
            }
        }

        @Override
        public void end(boolean interrupted) {
            climber.endHoming();
        }

        @Override
        public boolean isFinished() {
            return done;
        }
    }
}
