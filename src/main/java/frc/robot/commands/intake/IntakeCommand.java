package frc.robot.commands.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Utils;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    public IntakeCommand() {
        addRequirements(RobotContainer.intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        boolean isStuck = RobotContainer.intake.getCurrentOutput() > Constants.Intake.currentStuck;
        Logger.recordOutput("Intake/IsStuck", isStuck);
        if (isStuck) {
            System.out.println("> Detected: Intake is stuck. DUTY CYCLE: 80%");
            RobotContainer.intake.setPercent(0.8);
        }
        else {
            RobotContainer.intake.ballsIn();
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
