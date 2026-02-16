package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.calibration.ShooterRealityCalibrator;

/** Startet eine neue Session mit dem aktuell gewählten Calibration-Preset. */
public class RunCalibrationPresetCommand extends InstantCommand {
    public RunCalibrationPresetCommand(ShooterRealityCalibrator calibrator) {
        super(calibrator::startSelectedPreset);
    }
}

