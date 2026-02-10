package frc.robot;

import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
    public static final ClimberSubsystem climber;
    public static final Controls controls;

    static {
        climber = new ClimberSubsystem();
        controls = new Controls();
    }
}
