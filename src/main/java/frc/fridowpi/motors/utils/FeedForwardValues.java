package frc.fridowpi.motors.utils;

/**
 * FeedForwardValues
 */
public class FeedForwardValues {
	public double kS;
	public double kV;
	public double kA;

	public FeedForwardValues(double kS, double kV, double kA) {
		this.kS = kS;
		this.kV = kV;
		this.kA = kA;
	}

	public FeedForwardValues(double kS, double kV) {
		this.kS = kS;
		this.kV = kV;
		this.kA = 0;
	}

	public double getkS() {
		return kS;
	}

	public void setkS(double kS) {
		this.kS = kS;
	}

	public double getkV() {
		return kV;
	}

	public void setkV(double kV) {
		this.kV = kV;
	}

	public double getkA() {
		return kA;
	}

	public void setkA(double kA) {
		this.kA = kA;
	}

	public FeedForwardValues clone() {
		try {
			return (FeedForwardValues) super.clone();
		} catch (CloneNotSupportedException e) {
			FeedForwardValues copy = new FeedForwardValues(kS, kV, kA);
			return copy;
		}
	}
}

