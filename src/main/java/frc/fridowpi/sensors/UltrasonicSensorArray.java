package frc.fridowpi.sensors;

import frc.fridowpi.sensors.base.IUltrasonic;
import frc.fridowpi.sensors.base.IUltrasonicSensorArray;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class UltrasonicSensorArray implements IUltrasonicSensorArray, Sendable{
    private IUltrasonic rightSensor;
    private IUltrasonic leftSensor;
    private double sensorDistance;
    private LinearFilter distanceFilter;
    private LinearFilter angleFilter;

    /***
     * @param rightSensor instance of the right sensor
     * @param leftSensor instance of the left sensor
     * @param sensorDistance in mm
     * */
    public UltrasonicSensorArray(IUltrasonic rightSensor, IUltrasonic leftSensor, double sensorDistance) {
        this.leftSensor = leftSensor;
        this.rightSensor = rightSensor;
        this.sensorDistance = sensorDistance;

        distanceFilter = LinearFilter.movingAverage(8);
        distanceFilter.reset();

        angleFilter = LinearFilter.movingAverage(8);
        angleFilter.reset();

        SendableRegistry.addLW(this, "UltrasonicSensorArray", 0);
    }

    /***
     * @return raw Distance to the target in mm
     * */
    @Override
    public double getRawDistance() {
        return (leftSensor.getFilteredDistance() + rightSensor.getFilteredDistance()) / 2;
    }

    /***
     * @return "filtered Distance to the target in mm"
     * */
    @Override
    public double getFilteredDistance() {
        return distanceFilter.calculate(getRawDistance());
    }

    /***
     * @return "instance of the left sensor"
     * */
    @Override
    public IUltrasonic getLeftSensor() {
        return this.leftSensor;
    }

    /***
     * @return "instance of the right sensor"
     * */
    @Override
    public IUltrasonic getRightSensor() {
        return this.rightSensor;
    }

    /***
     * @return "raw angle to the target as Rotation2D"
     * */
    @Override
    public Rotation2d getRawAngle() {
        return new Rotation2d(Math.atan((rightSensor.getFilteredDistance() - leftSensor.getFilteredDistance()) / this.sensorDistance));
    }

    /***
     * @return "filtered angle to the target as Rotation2D"
     * */
    @Override
    public Rotation2d getFilteredAngle() {
        return new Rotation2d(angleFilter.calculate(getRawAngle().getRadians()));
    }

    /***
     * @return "distance between the sensors in the array in mm"
     * */
    @Override
    public double getSensorDistance() {
        return this.sensorDistance;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("rawDistance", this::getRawDistance, null);
        builder.addDoubleProperty("filteredDistance", this::getFilteredDistance, null);
        builder.addDoubleProperty("rawAngle", () -> this.getRawAngle().getDegrees(), null);
        builder.addDoubleProperty("filteredAngle", () -> this.getFilteredAngle().getDegrees(), null);
    }
}
