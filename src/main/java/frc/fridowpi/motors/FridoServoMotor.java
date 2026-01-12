package frc.fridowpi.motors;

import edu.wpi.first.wpilibj.Servo;

public class FridoServoMotor extends Servo {
    private double maxAngle = 180;

    public FridoServoMotor(int channel) {
        super(channel);
    }

    public void setMaxAngle(double angle) {
        this.maxAngle = angle;
    }

    @Override
    public void setAngle(double degrees) {
        super.setAngle(degrees*180/maxAngle);
    }

    @Override
    public double getAngle() {
        return super.getAngle()/180*maxAngle;
    }

	public void increaseAngle(double step) {
		setAngle(getAngle() + step);
	}
}
