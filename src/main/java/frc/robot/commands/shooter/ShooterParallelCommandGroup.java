package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.turret.SmartTurret;

// For Autonomous
public class ShooterParallelCommandGroup extends ParallelCommandGroup {
    public ShooterParallelCommandGroup() {
        super.addCommands(new ShootCommand(),
                          new SmartTurret()
        );
    }
}