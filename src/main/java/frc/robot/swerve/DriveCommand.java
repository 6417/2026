package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.fridowpi.utils.Vector2;
import frc.robot.RobotContainer;
import frc.robot.Controls.DriveSpeed;
import frc.robot.Constants;
import frc.robot.Controls;

public class DriveCommand extends Command {
    public DriveCommand(SwerveDrive drive) {
        addRequirements(drive);
    }

    public void execute() {
        var joystick = RobotContainer.controls.driveJoystick;

        var x = -joystick.getRawAxis(1);
        var y = -joystick.getRawAxis(0);
        
        var rot = -joystick.getRightX();

        // Apply deadband4
        x = applyDeadband(x, Controls.deadBandTurn);
        y = applyDeadband(y, Controls.deadBandTurn);
        rot = applyDeadband(rot, Controls.deadBandTurn);
        var xy = new Vector2(x, y);

        xy.scale(Constants.SwerveDrive.maxSpeed * RobotContainer.controls.speedFactors.get(RobotContainer.controls.getActiveSpeedFactor()));
        rot *= Constants.SwerveDrive.maxTurnSpeed;
        setChassisSpeeds(xy, rot);
    }

    public static Vector2 applyDeadband(Vector2 xy, double deadBand) {
        return xy.normalized().scaled(applyDeadband(xy.magnitude(), deadBand));
    }

    public static double applyDeadband(double x, double deadBand) {
        return Math.abs(x) < deadBand ? 0 : (Math.abs(x) - deadBand) / (1 - deadBand) * Math.signum(x);
    }

    private void setChassisSpeeds(Vector2 vxy, double vRot) {
        switch (RobotContainer.controls.driveOrientation) {
            case Forwards:
                RobotContainer.drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,
                        vRot, new Rotation2d(0.0)));
                break;
            case Backwards:
                RobotContainer.drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,
                        vRot, Rotation2d.fromRadians(Math.PI)));
                break;
            case FieldOriented:
                RobotContainer.drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,
                        vRot, RobotContainer.gyro.getRotation2d()));
                break;
        }
    }
}
