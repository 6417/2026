package frc.fridowpi.utils;

public class Matrix2 {
    private double[][] components;

    public Matrix2(double[][] components) {
        if (components.length != 2 || components[0].length != 2) {
            throw new IllegalArgumentException();
        }
        this.components = components;
    }

    private void check_index_and_throw(int idx) {
        if (idx >= 2 || idx < 0) {
            throw new IndexOutOfBoundsException();
        }
    }

    public double get(int i, int j) {
        check_index_and_throw(i);
        check_index_and_throw(j);
        return components[i][j];
    }

    public void set(int i, int j, double v) {
        check_index_and_throw(i);
        check_index_and_throw(j);
        components[i][j] = v;
    }

    public double[][] asArray() {
        return components;
    }

    public Matrix2 mul(Matrix2 other) {
        double[][] result = new double[2][2];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                double sum = 0.0;
                for (int k = 0; k < 2; k++) {
                    sum += this.components[i][k] * other.components[k][j];
                }
                result[i][j] = sum;
            }
        }
        return new Matrix2(result);
    }

    public double det() {
        return components[0][0] * components[1][1] -
                components[1][0] * components[0][0];
    }

    public Matrix2 smul(double s) {
        return new Matrix2(new double[][] { { s * components[0][0], s * components[0][1] },
                { s * components[0][0], s * components[0][1] } });
    }

    public static Matrix2 identiy() {
        return new Matrix2(new double[][] { { 1, 0 }, { 0, 1 } });
    }

    public Matrix2 inverse() {
        if (det() == 0.0) {
            throw new ArithmeticException();
        }

        return new Matrix2(new double[][] {
                { components[1][1], -components[0][1] },
                { -components[1][0], components[0][0] }
        }).smul(det());
    }

    public Matrix2 add(Matrix2 other) {
        double[][] result = new double[2][2];

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                result[i][j] = get(i, j) + other.get(i, j);
            }
        }

        return new Matrix2(result);
    }

    public Vector2 vmul(Vector2 v) {
        return new Vector2(v.x * components[0][0] + v.y * components[0][1],
                v.x * components[1][0] + v.y * components[1][1]);
    }

    public static Matrix2 rot(double angle) {
        return new Matrix2(
                new double[][] { { Math.cos(angle), -Math.sin(angle) }, { Math.sin(angle), Math.cos(angle) } });
    }
}
