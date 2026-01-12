package frc.fridowpi.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.*;

/**
 * This implementation is based on algorithm 11.1 from the book "Convex
 * Optimization" by Stephen Boyd and Lieven Vandenberghe
 * https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf. It solves general
 * Quadratically Constraint Quadratic Programs
 * of the form argmin_{x} f_0(x) s.t. f_i(x) <= 0 for i = 1,...,m.
 */
public class BarrierMethod<N extends Num> {
    public static interface Func<N extends Num> extends Function<Vector<N>, Double> {
    }

    public static interface Gradient<N extends Num> extends Function<Vector<N>, Vector<N>> {
    }

    public static interface Hessian<N extends Num> extends Function<Vector<N>, Matrix<N, N>> {
    }

    public BarrierMethod(Func<N> f0, Gradient<N> gradf0, Hessian<N> hessf0) {
        this.f0 = f0;
        this.gradf0 = gradf0;
        this.hessf0 = hessf0;
    }

    /**
     * This is algorithm 9.2 from the book "Convex Optimization" by Stephen Boyd and
     * Lieven Vandenberghe
     * https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf
     */
    public static <N extends Num> double backTrackingLineSearch(
            Vector<N> x,
            Vector<N> step,
            Func<N> f,
            Function<Vector<N>, Boolean> isFeasible,
            Vector<N> gradFx,
            double alpha,
            double beta) {
        assert 0 < alpha && alpha < 0.5 : "invalid value alpha";
        assert 0 < beta && beta < 1.0 : "invalid value beta";

        double t = 1.0;
        final double stepNorm = step.norm();
        final double EPS = 3e-16;

        while (!isFeasible.apply(x.plus(step.times(t)))) {
            t *= beta;
        }

        double fx = f.apply(x);

        while (f.apply(x.plus(step.times(t))) > fx + alpha * t * gradFx.dot(step)) {
            t *= beta;
            if (t < EPS * stepNorm) {
                return t;
            }
        }
        return t;
    }

    /**
     * Solves the optimization problem $argmin_{x} F(x)$
     * This implementation is based on algorithm 9.5 from the book "Convex
     * Optimization" by Stephen Boyd and Lieven Vandenberghe
     * https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf
     */
    public static <N extends Num> Vector<N> newtonMinimize(
            Func<N> F,
            Gradient<N> gradF,
            Hessian<N> d2F,
            Function<Vector<N>, Boolean> isFeasible,
            Vector<N> x,
            double eps,
            double alpha,
            double beta) {

        assert isFeasible.apply(x) : "initial guess must be feasible";

        double lambdaSq;
        Vector<N> step;
        Vector<N> gradFx;
        double t;

        while (true) {
            gradFx = gradF.apply(x);
            step = new Vector<N>(d2F.apply(x).solve(gradFx)).times(-1);
            lambdaSq = -gradFx.dot(step);

            if (lambdaSq / 2 <= eps)
                return x;

            t = backTrackingLineSearch(
                    x, step, F, isFeasible, gradFx, alpha, beta);
            x = x.plus(step.times(t));
        }
    }

    public double eps = 1e-4;
    public double alpha = 0.1;
    public double beta = 0.7;
    public double mu = 50.0;

    private Func<N> f0;
    private Gradient<N> gradf0;
    private Hessian<N> hessf0;
    private ArrayList<Func<N>> constrs = new ArrayList<Func<N>>();
    private ArrayList<Gradient<N>> gradConstrs = new ArrayList<Gradient<N>>();
    private ArrayList<Hessian<N>> hessConstrs = new ArrayList<Hessian<N>>();

    public void addConstr(Func<N> f, Gradient<N> gradf, Hessian<N> hessf) {
        constrs.add(f);
        gradConstrs.add(gradf);
        hessConstrs.add(hessf);
    }

    public boolean isFeasible(Vector<N> x) {
        int m = numConstrs();
        for (int i = 0; i < m; i++) {

            if (constrs.get(i).apply(x) > 0)
                return false;
        }
        return true;
    }

    public int numConstrs() {
        int m = constrs.size();
        assert m == gradConstrs.size() : "gradConstrs size mismatch";
        assert m == hessConstrs.size() : "hessConstrs size mismatch";
        return m;
    }

    public Vector<N> solve(Vector<N> x) {
        int m = numConstrs();
        Double[] t = new Double[] { m / eps }; // enable capture in lambda below
        Func<N> aux = (Vector<N> y) -> {
            return t[0] * f0.apply(y) + phi(y);
        };

        Gradient<N> gradAux = (Vector<N> y) -> {
            return gradf0.apply(y).times(t[0]).plus(gradPhi(y));
        };

        Hessian<N> hessAux = (Vector<N> y) -> {
            return hessf0.apply(y).times(t[0]).plus(hessPhi(y));
        };

        while (true) {
            x = newtonMinimize(
                    aux, gradAux, hessAux, this::isFeasible,
                    x, eps, alpha, beta);
            if (m / t[0] < eps)
                return x;

            t[0] *= mu;
        }
    }

    private double phi(Vector<N> x) {
        int m = numConstrs();

        double val = 0.0;

        for (int i = 0; i < m; i++) {
            double fi = constrs.get(i).apply(x);
            if (fi >= 0)
                throw new Error("Constraint is not satisfied");
            val += -Math.log(-fi);
        }

        return val;
    }

    private Vector<N> gradPhi(Vector<N> x) {
        int m = numConstrs();

        Vector<N> val = null;
        for (int i = 0; i < m; i++) {
            double fi = constrs.get(i).apply(x);
            if (fi >= 0)
                throw new Error("Constraint is not satisfied");
            if (val != null) {
                val = val.plus(gradConstrs.get(i).apply(x).times(1.0 / (-fi)));
            } else {
                val = gradConstrs.get(i).apply(x).times(1.0 / (-fi));
            }
        }

        return val;
    }

    private Matrix<N, N> hessPhi(Vector<N> x) {
        int m = numConstrs();
        Matrix<N, N> val = null; // No clue how to instantiate a zero matrix of size N

        for (int i = 0; i < m; i++) {
            double fi = constrs.get(i).apply(x);
            if (fi >= 0)
                throw new Error("Constraint is not satisfied");
            Vector<N> gradFi = gradConstrs.get(i).apply(x);
            if (val != null) {
                val = val.plus(gradFi.times(gradFi.transpose()).times(1.0 / (fi * fi)))
                        .plus(hessConstrs.get(i).apply(x).times(1.0 / (-fi)));
            } else {
                val = gradFi.times(gradFi.transpose()).times(1.0 / (fi * fi))
                        .plus(hessConstrs.get(i).apply(x).times(1.0 / (-fi)));
            }
        }

        return val;
    }
}
