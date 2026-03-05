package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ManualClimberControl extends Command {
    private final DoubleSupplier percentSupplier;

    public ManualClimberControl(DoubleSupplier percentSupplier) {
        // Supplier allows live joystick input without storing controller here.
        this.percentSupplier = percentSupplier;
        addRequirements(RobotContainer.climber);
    }

    @Override
    public void execute() {
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
