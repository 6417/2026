package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Utils {
    public static double normalizeAngleRad(double angle) {
        return Math.asin(Math.sin(angle));
    }

    public static double wrap(double x) {
        return x - Math.floor(x + 0.5);
    }

    /** If there is no Alliance, returns false */
    public static boolean isRobotInNeutralZone() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }
        return (alliance.get() == Alliance.Blue
                && RobotContainer.drive.getPose().getX() > Constants.Field.neutralZoneStartX) ||
                (alliance.get() == Alliance.Red
                        && RobotContainer.drive.getPose().getX() < Constants.Field.neutralZoneStartX);

    }
}
