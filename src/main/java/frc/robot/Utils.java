package frc.robot;

public class Utils {
    public static double normalizeAngleRad(double angle) {
        return Math.asin(Math.sin(angle));
    }

    public static double wrap(double x) {
        return x - Math.floor(x + 0.5);
    }
}
