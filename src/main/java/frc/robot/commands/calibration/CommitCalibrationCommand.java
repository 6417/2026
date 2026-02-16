package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.calibration.ShooterRealityCalibrator;

/** Übernimmt die berechnete Suggestion und speichert sie als Runtime-Calibration. */
public class CommitCalibrationCommand extends InstantCommand {
    public CommitCalibrationCommand(ShooterRealityCalibrator calibrator) {
        super(calibrator::commitSuggestion);
    }
}

