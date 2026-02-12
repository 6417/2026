package frc.robot.commands.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Simulation-only scenario command:
 * moves the robot at constant field-relative velocity and fires at fixed intervals.
 */
public class SimMovingShotScenarioCommand extends Command {
    private final ShooterSubsystem shooter;
    private final Translation2d constantVelocityFieldMps;

    private Pose2d simPoseField;
    private double lastTimestampSec;
    private double nextShotTimestampSec;

    public SimMovingShotScenarioCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.constantVelocityFieldMps = Constants.ShooterSim.scenarioVelocityFieldMps;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        simPoseField = new Pose2d(
                Constants.ShooterSim.scenarioStartPositionField,
                new Rotation2d(Constants.ShooterSim.scenarioStartHeadingRad));
        shooter.setSimRobotState(simPoseField, constantVelocityFieldMps);

        lastTimestampSec = Timer.getFPGATimestamp();
        nextShotTimestampSec = lastTimestampSec + Constants.ShooterSim.scenarioShotIntervalSec;
    }

    @Override
    public void execute() {
        double nowSec = Timer.getFPGATimestamp();
        double dtSec = nowSec - lastTimestampSec;
        lastTimestampSec = nowSec;

        // Integrate constant field velocity.
        Translation2d delta = constantVelocityFieldMps.times(dtSec);
        simPoseField = new Pose2d(simPoseField.getTranslation().plus(delta), simPoseField.getRotation());

        shooter.setSimRobotState(simPoseField, constantVelocityFieldMps);
        shooter.shootWhileMoving(simPoseField, constantVelocityFieldMps);

        if (nowSec >= nextShotTimestampSec) {
            shooter.simulateFire();
            nextShotTimestampSec += Constants.ShooterSim.scenarioShotIntervalSec;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return !RobotBase.isSimulation();
    }
}
