package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RelaseChuchichaestliAndHomeRelativeEncoder extends Command {

    public RelaseChuchichaestliAndHomeRelativeEncoder() {
        // Supplier allows live joystick input without storing controller here.
        addRequirements(RobotContainer.climber);
    }

    @Override
    public void initialize(){
        RobotContainer.climber.startHoming();
    }

    @Override
    public void execute() {
        if (RobotContainer.climber.isMotorBlocked()){
            RobotContainer.climber.endHoming();
            end(false);
        }

        // Pass through manual input each cycle.
        // RobotContainer.climber.setManualPercent(percentSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop motor on release or interruption.
        RobotContainer.climber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
