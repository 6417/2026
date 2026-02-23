package frc.robot.commands.indexer;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * Prepares the indexer for shooter handoff.
 *
**/
public class IndexerCommand extends Command {
    private final IndexerSubsystem indexer;

    public IndexerCommand() {
        this.indexer = RobotContainer.indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        indexer.setIndexerPercent(Constants.Indexer.indexingRpm);;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
