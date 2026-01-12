package frc.fridowpi.sensors;

import frc.fridowpi.sensors.base.IAnalogLightbarrier;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogLightbarrier implements IAnalogLightbarrier, Sendable {
    private AnalogInput input;
    private double threshold;

    public AnalogLightbarrier(int channel, double threshold) {
        this.input = new AnalogInput(channel);
        this.threshold = threshold;

        SendableRegistry.addLW(this, "AnalogLightbarrier", channel);
    }

    @Override
    public double getRawVoltage() {
        return input.getVoltage();
    }

    @Override
    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }

    @Override
    public boolean isTriggered() {
        return getRawVoltage() >= this.threshold;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
       builder.addDoubleProperty("Voltage", this::getRawVoltage, null);
       builder.addBooleanProperty("Triggered", this::isTriggered, null);
    }
}
