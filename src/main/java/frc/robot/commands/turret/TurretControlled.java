package frc.robot.commands.turret;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class TurretControlled extends Command {
    private double input;

    public TurretControlled() {
        addRequirements(RobotContainer.turret);
    }

    @Override
    public void execute() {
        input = RobotContainer.controls.getJoystickAxesFromOperatorJoystick()[2];
        input = applyDeadband(input, 0.15);

        RobotContainer.turret.setPercent(-input * 0.1); // 0.5 is max speed factor for manual control, can be tuned
    }

    private static double applyDeadband(double x, double deadBand) {
        return Math.abs(x) < deadBand ? 0 : (Math.abs(x) - deadBand) / (1 - deadBand) * Math.signum(x);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.controls.isTurretAutomated();
    }
}