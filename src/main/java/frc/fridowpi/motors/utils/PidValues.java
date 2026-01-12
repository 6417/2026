package frc.fridowpi.motors.utils;

import java.util.Optional;

public class PidValues {
    public Optional<Double> kF = Optional.empty();
    public Optional<Integer> slotIdX = Optional.empty();
    public Optional<Double> upperSpeedLimit = Optional.empty();
    public Optional<Double> lowerSpeedLimit = Optional.empty();
    public double kP;
    public double kI;
    public double kD;
    public double peakOutputReverse = -1;
    public double peakOutputForward = 1;
    public Optional<Double> cruiseVelocity = Optional.empty();
    public Optional<Double> acceleration = Optional.empty();
    public Optional<Double> tolerance = Optional.empty();
	// TODO IZone
    public Optional<Double> iZone = Optional.empty();

    public PidValues(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PidValues(double kP, double kI, double kD, double peakOutputReverse, double peakOutputForward) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.peakOutputReverse = peakOutputReverse;
        this.peakOutputForward = peakOutputForward;
    }

    public PidValues(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        this.kF = Optional.of(kF);
    }

    public PidValues(double kP, double kI, double kD, double kF, double peakOutputReverse, double peakOutputForward) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        this.kF = Optional.of(kF);
        this.peakOutputReverse = peakOutputReverse;
    }

    /**
     * @param cruiseVelocity the cruiseVelocity to set
     */
    public void setCruiseVelocity(double cruiseVelocity) {
        this.cruiseVelocity = Optional.of(cruiseVelocity);
    }

    /**
     * @param acceleration the acceleration to set
     */
    public void setAcceleration(double acceleration) {
        this.acceleration = Optional.of(acceleration);
    }

    public void setTolerance(double tolerance) {
        this.tolerance = Optional.of(tolerance);
    }

    @Override
    public PidValues clone() {
        try {
            return (PidValues) super.clone();
        } catch (CloneNotSupportedException e) {
            PidValues copy = new PidValues(kP, kI, kD, peakOutputReverse, peakOutputForward);
            kF.ifPresent((Double kF) -> copy.kF = Optional.of((double) kF)); // deep copying optional
            cruiseVelocity
                    .ifPresent((Double cruiseVelocity) -> copy.cruiseVelocity = Optional.of((double) cruiseVelocity)); // deep
                                                                                                                       // copying
                                                                                                                       // optional
            acceleration.ifPresent((Double acceleration) -> copy.acceleration = Optional.of((double) acceleration)); // deep
                                                                                                                     // copying
                                                                                                                     // optional
            return copy;
        }
    }

	public void setIZone(double iZone) {
	}

	public PidValues withIZone(double iZone) {
		var copy = this.clone();
		copy.setIZone(iZone);
		return copy;
	}
}
