package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.CalculationSubsystem.ShootingMode;
import frc.robot.RobotContainer;

public class SmartTurret extends Command {
    public SmartTurret() {
        addRequirements(RobotContainer.turret);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        Rotation2d desiredAngle = RobotContainer.calculationSubsystem.getDesiredTurretAngle();

        if (RobotContainer.calculationSubsystem.getShootingMode() == ShootingMode.MODE_STATIONARY_TURRETFIX || RobotContainer.calculationSubsystem.getShootingMode() == ShootingMode.MODE_FIXED) {
            return;
        }
        RobotContainer.turret.setDesiredRotation(desiredAngle);
    }

    @Override
    public void end(boolean interrupted){
        RobotContainer.turret.stopRotationMotor();
    }

    @Override
    public boolean isFinished() {
        return !RobotContainer.controls.isTurretAutomated();
    }
}
