package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class SmartTurret extends Command {
    private final TurretSubsystem turret;

    public SmartTurret(TurretSubsystem turret) {
        this.turret = turret;
         
        addRequirements(turret); // don't add drive
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        Rotation2d desiredAngle = RobotContainer.calculationSubsystem.getTurretAngle();
        turret.setDesiredRotation(desiredAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
