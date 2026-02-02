package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SmartTurret extends Command {
    private final TurretSubsystem turret;
    private final SwerveSubsystem drive;

    public SmartTurret(TurretSubsystem turret, SwerveSubsystem drive) {
       
        // Command implementation goes here
        this.drive = drive;
        this.turret = turret;
         
        addRequirements(turret); // don't add drive
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
