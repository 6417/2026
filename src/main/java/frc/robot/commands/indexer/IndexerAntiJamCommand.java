package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Runs the indexer with beam-break based anti-jam behavior.
 *
 * Normal behavior:
 * - run indexer forward at feed RPM
 *
 * Jam behavior:
 * - if beam-break state does not change for a configured timeout,
 *   reverse indexer for a short duration, then continue feeding forward.
 */
public class IndexerAntiJamCommand extends Command {
    private enum Mode {
        FEED_FORWARD,
        UNJAM_REVERSE
    }

    private final double feedRpm;
    private final double reverseRpm;
    private final double noTransitionTimeoutSec;
    private final double reverseDurationSec;

    private Mode mode = Mode.FEED_FORWARD;
    private boolean previousBeamState;
    private boolean hasSeenTransition;
    private double lastTransitionTimestampSec;
    private double reverseStartTimestampSec;

    public IndexerAntiJamCommand() {
        this(
                Constants.Indexer.feedRpm,
                Constants.Indexer.unjamReverseRpm,
                Constants.Indexer.jamNoTransitionTimeoutSec,
                Constants.Indexer.unjamReverseDurationSec);
    }

    public IndexerAntiJamCommand(
            double feedRpm,
            double reverseRpm,
            double noTransitionTimeoutSec,
            double reverseDurationSec) {
        this.feedRpm = feedRpm;
        this.reverseRpm = reverseRpm;
        this.noTransitionTimeoutSec = noTransitionTimeoutSec;
        this.reverseDurationSec = reverseDurationSec;
        addRequirements(RobotContainer.indexer);
    }

    @Override
    public void initialize() {
        mode = Mode.FEED_FORWARD;
        previousBeamState = RobotContainer.indexer.isBallDetected();
        hasSeenTransition = false;
        lastTransitionTimestampSec = Timer.getFPGATimestamp();
        reverseStartTimestampSec = 0.0;
    }

    @Override
    public void execute() {
        double nowSec = Timer.getFPGATimestamp();
        boolean beamState = RobotContainer.indexer.isBallDetected();

        if (beamState != previousBeamState) {
            previousBeamState = beamState;
            hasSeenTransition = true;
            lastTransitionTimestampSec = nowSec;
        }

        if (mode == Mode.FEED_FORWARD) {
            RobotContainer.indexer.run(feedRpm);

            // Start anti-jam only after we saw at least one ball transition.
            if (hasSeenTransition && (nowSec - lastTransitionTimestampSec) >= noTransitionTimeoutSec) {
                mode = Mode.UNJAM_REVERSE;
                reverseStartTimestampSec = nowSec;
            }
            return;
        }

        RobotContainer.indexer.run(reverseRpm);
        if ((nowSec - reverseStartTimestampSec) >= reverseDurationSec) {
            mode = Mode.FEED_FORWARD;
            lastTransitionTimestampSec = nowSec;
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
