package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.calibration.ShooterRealityCalibrator;

public class CancelCalibrationCommand extends InstantCommand {
    public CancelCalibrationCommand(ShooterRealityCalibrator calibrator) {
        super(calibrator::cancelSession);
    }
}

