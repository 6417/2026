package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.calibration.ShooterRealityCalibrator;

public class RunCalibrationPresetCommand extends InstantCommand {
    public RunCalibrationPresetCommand(ShooterRealityCalibrator calibrator) {
        super(calibrator::startSelectedPreset);
    }
}

