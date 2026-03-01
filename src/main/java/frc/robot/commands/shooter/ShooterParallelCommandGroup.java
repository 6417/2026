package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.turret.SmartTurret;

public class ShooterParallelCommandGroup extends ParallelCommandGroup {
    public ShooterParallelCommandGroup() {
        super.addCommands(new ShootCommand(),
        new ServoCommand(),
        new PulseFeederCommand(),
        new SmartTurret(RobotContainer.turret)
        );
    }
}