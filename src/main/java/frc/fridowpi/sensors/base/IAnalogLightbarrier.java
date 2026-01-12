package frc.fridowpi.sensors.base;

public interface IAnalogLightbarrier {
    public double getRawVoltage();

    public void setThreshold(double threshold);

    public boolean isTriggered();
}
