
package frc.fridowpi.utils;

/**
 * Vector3
 */
public class Vector3 {

	public double x;
	public double y;
	public double z;

	public Vector3(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public Vector3 add(Vector3 other) {
		return new Vector3(x + other.x, y + other.y, z + other.z);
	}

	public Vector3 sub(Vector3 other) {
		return new Vector3(x - other.x, y - other.y, z - other.z);
	}

	public Vector3 scale(double s) {
		return new Vector3(s * x, s * y, s * z);
	}

	public Vector3 mulElementWise(double s) {
		return new Vector3(s * x, s * y, s * z);
	}

	public double dot(Vector3 other) {
		return x * other.x + y * other.y + z * other.z;
	}

	public Vector3 cross(Vector3 other) {
		return new Vector3(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
	}

	public double magnitude() {
		return Math.sqrt(x * x + y * y + z * z);
	}

	public Vector3 normalized() {
		double mag = magnitude();
		if (mag == 0) {
			return new Vector3(0, 0, 0);
		}
		return new Vector3(x / mag, y / mag, z / mag);
	}

	@Override
	public String toString() {
		return String.format("{ %f, %f, %f }", x, y, z);
	}
}
