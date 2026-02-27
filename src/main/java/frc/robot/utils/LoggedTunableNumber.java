package frc.robot.utils;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import frc.robot.Constants;

/**
 * A number that can be tuned live from AdvantageScope during TUNING_MODE,
 * and falls back to a hardcoded default in competition mode.
 *
 * Published to the "/Tuning/<key>" NT path, which AdvantageScope exposes
 * as an editable field when tuning mode is active (the slider icon in the toolbar).
 *
 * Usage:
 *   private final LoggedTunableNumber tuneTopRpm =
 *       new LoggedTunableNumber("Shooter/TuneTopRPM", 3000);
 *
 *   // In periodic or command:
 *   double rpm = tuneTopRpm.get();
 */
public class LoggedTunableNumber {
    private final double defaultValue;
    private final LoggedNetworkNumber networkNumber;

    public LoggedTunableNumber(String key, double defaultValue) {
        this.defaultValue = defaultValue;
        this.networkNumber = new LoggedNetworkNumber("/Tuning/" + key, defaultValue);
    }

    /**
     * Returns the current value.
     * In TUNING_MODE: reads live from the "/Tuning/<key>" NT entry (editable in AdvantageScope).
     * Otherwise: returns the hardcoded default.
     */
    public double get() {
        if (Constants.TUNING_MODE) {
            return networkNumber.get();
        }
        return defaultValue;
    }
}
