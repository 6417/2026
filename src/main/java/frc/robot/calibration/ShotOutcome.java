package frc.robot.calibration;

/**
 * Operator-Label für ein erfasstes Kalibrier-Sample.
 *
 * <p>HIT/MISS_UNKNOWN sind grobe Labels, die Richtungslabels dienen zusätzlich
 * der gezielten Korrektur von Turret-Offset bzw. Distanz-Bias.
 */
public enum ShotOutcome {
    HIT,
    MISS_UNKNOWN,
    MISS_LEFT,
    MISS_RIGHT,
    MISS_SHORT,
    MISS_LONG
}

