package frc.fridowpi.utils;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class AccelerationLimiter {
    public double maxAccel;
    public double radius;
    public double mass = 60.0;

    public double barrierAlpha = 0.1;
    public double barrierBeta = 0.7;
    public double barrierMu = 50.0;
    public double barrierEps = 1e-3;

    // specifies how the different control directions (x, y, theta) are
    // weighted in the optimization least squares problem.
    public Matrix<N3, N3> minizationWeights = MatBuilder.fill(Nat.N3(), Nat.N3(),
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0);

    public AccelerationLimiter(double maxAccel, double radius) {
        this.maxAccel = maxAccel;
        this.radius = radius;
    }

    private BarrierMethod.Func<N3> makeObjective(
            Vector<N3> desiredVel) {
        return (Vector<N3> vel) -> {
            Vector<N3> error = desiredVel.minus(vel);
            return error.transpose().times(minizationWeights.times(error)).get(0, 0);
        };
    }

    private BarrierMethod.Gradient<N3> objectiveGradient(Vector<N3> desiredVel) {
        return (Vector<N3> vel) -> {
            Vector<N3> error = desiredVel.minus(vel);
            return new Vector<N3>(minizationWeights.times(error).times(-2));
        };
    }

    private BarrierMethod.Hessian<N3> objectiveHess() {
        return (Vector<N3> vel) -> {
            return minizationWeights.times(2);
        };
    }

    /**
     * The following calculations are based on this python script and were then 
     * ported to java using ChatGPT.
     *
     * import sympy as sp
     * 
     * vx, vy, cvx, cvy, omega, omegaCurr, rx, ry, dt = sp.symbols('vx vy cvx cvy
     * omega omegaCurr rx ry dt')
     * 
     * vel = sp.Matrix([vx, vy])
     * currVel = sp.Matrix([cvx, cvy])
     * 
     * alpha = (omega - omegaCurr) / dt
     * 
     * constr = ((vel - currVel) / dt + sp.Matrix([-alpha * ry, alpha * rx])).dot(
     * (vel - currVel) / dt + sp.Matrix([-alpha * ry, alpha * rx])
     * )
     * 
     * gradient = sp.simplify(sp.Matrix([sp.diff(constr, var) for var in [vx, vy,
     * omega]]))
     * hessian = sp.simplify(sp.Matrix([[sp.diff(gradient[i], var) for var in [vx,
     * vy, omega]] for i in range(3)]))
     * 
     * print("Gradient:")
     * sp.pprint(gradient)
     * print("\nHessian:")
     * sp.pprint(hessian)
     */
    private BarrierMethod.Func<N3> accelConstraint(Vector<N2> r,
            Vector<N3> currentVel,
            double dt) {
        return (Vector<N3> vel) -> {
            double omegaCurr = currentVel.get(2);
            double omega = vel.get(2);
            double alpha = (omega - omegaCurr) / dt;

            Vector<N2> accel = VecBuilder.fill(
                    vel.get(0) - currentVel.get(0),
                    vel.get(1) - currentVel.get(1)).times(1.0 / dt);

            Vector<N2> val = accel.plus(VecBuilder.fill(
                    -alpha * r.get(1),
                    alpha * r.get(0)));

            return val.dot(val) - maxAccel * maxAccel;
        };
    }

    private BarrierMethod.Gradient<N3> gradAccelConstraint(
            Vector<N2> r,
            Vector<N3> currentVel,
            double dt) {
        return (Vector<N3> vel) -> {
            // ChatGPT generated, based on calculations in sympy
            double cvx = currentVel.get(0);
            double cvy = currentVel.get(1);
            double omegaCurr = currentVel.get(2);
            double vx = vel.get(0);
            double vy = vel.get(1);
            double omega = vel.get(2);
            double rx = r.get(0);
            double ry = r.get(1);

            double gradX = -((2 * (cvx + omega * ry - omegaCurr * ry - vx)) / (dt * dt));
            double gradY = (2 * (-cvy + omega * rx - omegaCurr * rx + vy)) / (dt * dt);
            double gradOmega = (2 * (-cvy * rx - omegaCurr * rx * rx + cvx * ry - omegaCurr * ry * ry +
                    omega * (rx * rx + ry * ry) - ry * vx + rx * vy)) / (dt * dt);

            return VecBuilder.fill(gradX, gradY, gradOmega);
        };
    }

    private BarrierMethod.Hessian<N3> hessAccelConstraint(
            Vector<N2> r,
            Vector<N3> currentVel,
            double dt) {

        return (Vector<N3> vel) -> {
            // ChatGPT generated, based on calculations in sympy
            double rx = r.get(0);
            double ry = r.get(1);

            double h11 = 2 / (dt * dt);
            double h22 = 2 / (dt * dt);
            double h33 = 0;
            double h13 = -((2 * ry) / (dt * dt));
            double h23 = (2 * rx) / (dt * dt);

            return MatBuilder.fill(Nat.N3(), Nat.N3(), h11, 0, h13, 0, h22, h23, h13, h23, h33);
        };
    }

    public ChassisSpeeds constrain(ChassisSpeeds currentSpeed,
            ChassisSpeeds desiredSpeed,
            double dt) {
        final Vector<N3> currentVel = VecBuilder.fill(currentSpeed.vxMetersPerSecond,
                currentSpeed.vyMetersPerSecond,
                currentSpeed.omegaRadiansPerSecond);

        final Vector<N3> desiredVel = VecBuilder.fill(desiredSpeed.vxMetersPerSecond,
                desiredSpeed.vyMetersPerSecond,
                desiredSpeed.omegaRadiansPerSecond);

        var obj = makeObjective(desiredVel);
        var gradObj = objectiveGradient(desiredVel);
        var hessObj = objectiveHess();
        BarrierMethod<N3> solver = new BarrierMethod<>(obj, gradObj, hessObj);
        solver.alpha = barrierAlpha;
        solver.beta = barrierBeta;
        solver.eps = barrierEps;
        solver.mu = barrierMu;

        Vector<N2> rVec = VecBuilder.fill(Math.sqrt(2) * radius, Math.sqrt(2) * radius);

        for (int i = 0; i < 4; i++) {
            Matrix<N2, N2> rotMat = MatBuilder.fill(Nat.N2(), Nat.N2(),
                    Math.cos(0.5 * Math.PI * i), -Math.sin(0.5 * Math.PI * i),
                    Math.sin(0.5 * Math.PI * i), Math.cos(0.5 * Math.PI * i));

            Vector<N2> r = new Vector<N2>(rotMat.times(rVec));
            var constr = accelConstraint(r, currentVel, dt);
            var constrGrad = gradAccelConstraint(r, currentVel, dt);
            var constrHess = hessAccelConstraint(r, currentVel, dt);

            solver.addConstr(constr, constrGrad, constrHess);
        }

        Vector<N3> constrainedVel = solver.solve(currentVel); // current speed should always be feasible

        ChassisSpeeds constrainedSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0));
        constrainedSpeed.vxMetersPerSecond = constrainedVel.get(0);
        constrainedSpeed.vyMetersPerSecond = constrainedVel.get(1);
        constrainedSpeed.omegaRadiansPerSecond = constrainedVel.get(2);

        return constrainedSpeed;
    }
}
