package frc.fridowpi.sensors.base;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IUltrasonicSensorArray extends IUltrasonic {
    public IUltrasonic getLeftSensor();

    public IUltrasonic getRightSensor();

    public Rotation2d getRawAngle();

    public Rotation2d getFilteredAngle();

    public double getSensorDistance();
}