package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.commands.climber.ManualClimberControl;
import frc.robot.commands.climber.ClimberZeroCommand;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;

public class Controls {
    private final CommandXboxController operatorController =
            new CommandXboxController(Constants.Joystick.operatorJoystickId);

    public Controls() {
        // Operator presets for climber states (A/B/Y).
        operatorController.a().onTrue(new ClimberCommand(ClimberState.LOW));
        operatorController.b().onTrue(new ClimberCommand(ClimberState.MID));
        operatorController.y().onTrue(new ClimberCommand(ClimberState.HIGH));

        // Manual jog while right bumper is held (left Y axis).
        operatorController.rightBumper().whileTrue(
                new ManualClimberControl(() -> -operatorController.getLeftY()));

        // Zero routine: drive into hard-stop until current threshold, then set encoder zero.
        operatorController.x().onTrue(new ClimberZeroCommand());
    }
}
