package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.climber.RelaseChuchichaestliAndHomeRelativeEncoderCommand;

public class ZeroGroup extends ParallelCommandGroup {
    public ZeroGroup() {
        super.addCommands(
            new TurretZeroCommand(RobotContainer.turret),
            new RelaseChuchichaestliAndHomeRelativeEncoderCommand(RobotContainer.climber)
        );
    }
}