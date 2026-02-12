package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;

public class TurretControlled extends Command {
    private final TurretSubsystem turret;

    private double input;

    public TurretControlled(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        input = RobotContainer.controls.getJoystickAxes()[2];
        input = applyDeadband(input, 0.15);

        turret.setPercent(input * 0.1);
    }

    private static double applyDeadband(double x, double deadBand) {
        return Math.abs(x) < deadBand ? 0 : (Math.abs(x) - deadBand) / (1 - deadBand) * Math.signum(x);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.controls.isTurretAutomated();
    }
}