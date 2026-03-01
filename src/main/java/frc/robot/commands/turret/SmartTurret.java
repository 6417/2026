package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.RobotContainer;

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
        Rotation2d desiredAngle = RobotContainer.calculationSubsystem.getDesiredTurretAngle();
        turret.setDesiredRotation(desiredAngle);
    }

    @Override
    public void end(boolean interrupted){
        turret.stopRotationMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
