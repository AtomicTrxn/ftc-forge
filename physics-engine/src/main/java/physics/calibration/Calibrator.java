package physics.calibration;

import physics.MotorModel;
import physics.MotorSpec;
import physics.MecanumKinematics;
import java.util.ArrayList;

import java.util.List;
import java.util.Map;

/**
 * R5's staged calibration fit, revised (per the Opus review) from a single undifferentiated
 * grid search into stages that can actually separate their parameters:
 *   1. Battery (R_battery, V_internal): closed-form linear regression -- no search needed.
 *   2. Motor friction (tauStatic, viscousB): one-step-ahead prediction, resetting to the
 *      recorded state each row (not open-loop replay over the whole run, which would let
 *      position error compound and produce a misleading fit).
 *   3. Thermal: not implemented in this pass -- R5's own design only calls for fitting it
 *      "if long-run residuals justify it," and this phase's validation runs are short.
 */
public class Calibrator {

    public static class BatteryFit {
        public final double vInternal;
        public final double rBattery;
        public BatteryFit(double vInternal, double rBattery) { this.vInternal = vInternal; this.rBattery = rBattery; }
    }

    /**
     * Stage 1: linear least-squares fit of V = Vinternal - Rbattery * Itotal. Closed-form, no search.
     *
     * Itotal here is the duty-scaled battery-rail current (|power| * |current| per motor, summed),
     * matching BatteryModel's own closed-form derivation exactly -- not the raw per-motor
     * current alone. Using raw |current| instead systematically biases the fit whenever any
     * motor runs below full power, since a real battery only sees a duty-cycle-averaged share
     * of each motor's coil current, not the full instantaneous value (this was caught by a
     * failing recovery test against a synthetic ground-truth log, not assumed correct).
     */
    public static BatteryFit fitBattery(List<TelemetryLogRow> rows) {
        int n = rows.size();
        if (n < 30) throw new IllegalArgumentException("Need at least 30 battery samples");
        double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
        for (TelemetryLogRow row : rows) {
            double iTotal = 0;
            for (TelemetryLogRow.MotorSample s : row.motors.values())
                iTotal += Math.abs(s.power) * Math.abs(s.currentA);
            double v = row.batteryVoltageV;
            sumX += iTotal;
            sumY += v;
            sumXY += iTotal * v;
            sumXX += iTotal * iTotal;
        }
        double denominator = n * sumXX - sumX * sumX;
        if (denominator < 1e-6) throw new IllegalArgumentException("Battery fit needs varied current load");
        double slope = (n * sumXY - sumX * sumY) / denominator;
        double intercept = (sumY - slope * sumX) / n;
        if (!Double.isFinite(intercept) || intercept <= 0 || intercept > 18
            || !Double.isFinite(slope) || slope > 0 || slope < -2)
            throw new IllegalArgumentException("Battery fit is outside physical bounds; check voltage/current columns");
        return new BatteryFit(intercept, -slope);
    }

    public static class FrictionFit {
        public final double tauStaticNm;
        public final double viscousBNms;
        public final double residualSumSquares;
        public FrictionFit(double tauStaticNm, double viscousBNms, double residualSumSquares) {
            this.tauStaticNm = tauStaticNm;
            this.viscousBNms = viscousBNms;
            this.residualSumSquares = residualSumSquares;
        }
    }

    /**
     * Stage 2: grid search over (tauStatic, viscousB) minimizing one-step-ahead velocity
     * prediction error for a single named motor, resetting to the recorded omega each row
     * (per the Opus review's correction -- not open-loop replay across the whole run).
     */
    public static FrictionFit fitMotorFriction(List<TelemetryLogRow> rows, String motorName, MotorSpec spec,
                                                double rotationalInertiaKgM2) {
        if (!Double.isFinite(rotationalInertiaKgM2) || rotationalInertiaKgM2 <= 0)
            throw new IllegalArgumentException("Motor load inertia must be positive and measured/estimated");
        if (rows.size() < 30) throw new IllegalArgumentException("Need at least 30 motor samples");
        double bestTauStatic = 0, bestViscousB = 0, bestError = Double.MAX_VALUE;

        for (double tauStatic = 0.0; tauStatic <= 0.4; tauStatic += 0.02) {
            for (double viscousB = 0.0; viscousB <= 0.02; viscousB += 0.001) {
                MotorModel model = new MotorModel();
                model.tauStaticNm = tauStatic;
                model.viscousBNms = viscousB;

                double error = 0;
                int count = 0;
                for (int i = 0; i + 1 < rows.size(); i++) {
                    TelemetryLogRow.MotorSample cur = rows.get(i).motors.get(motorName);
                    TelemetryLogRow.MotorSample next = rows.get(i + 1).motors.get(motorName);
                    if (cur == null || next == null) continue;

                    double dtSeconds = (rows.get(i + 1).tMs - rows.get(i).tMs) / 1000.0;
                    if (dtSeconds <= 0) continue;

                    double omegaNow = (cur.velTps / spec.encoderCountsPerRev) * 2 * Math.PI;
                    double vActual = cur.power * rows.get(i).batteryVoltageV;
                    double tau = model.torque(spec, omegaNow, vActual);
                    double predictedOmegaNext = omegaNow + (tau / rotationalInertiaKgM2) * dtSeconds;

                    double actualOmegaNext = (next.velTps / spec.encoderCountsPerRev) * 2 * Math.PI;
                    double diff = predictedOmegaNext - actualOmegaNext;
                    error += diff * diff;
                    count++;
                }
                if (count > 0 && error < bestError) {
                    bestError = error;
                    bestTauStatic = tauStatic;
                    bestViscousB = viscousB;
                }
            }
        }
        if (bestError == Double.MAX_VALUE) throw new IllegalArgumentException("No paired motor samples for " + motorName);
        return new FrictionFit(bestTauStatic, bestViscousB, bestError);
    }

    private record DrivePair(double dt, double vx, double vy, double omega,
                             double targetVx, double targetVy, double targetOmega,
                             double nextVx, double nextVy, double nextOmega) { }
    public record DriveFit(double responseTimeS, double maxAccelMps2, double yawResponseTimeS,
                           double maxYawAccelRadps2, double translationRmseMps, double yawRmseRadps,
                           int samples) { }

    /** Fits the fixed-tick planar controller from measured chassis velocities and wheel encoders. */
    public static DriveFit fitDrive(List<TelemetryLogRow> rows, Map<String, MotorSpec> specs,
                                    double wheelRadiusM, double trackWidthM, double wheelBaseM) {
        if (!Double.isFinite(wheelRadiusM) || wheelRadiusM <= 0 || !Double.isFinite(trackWidthM)
            || trackWidthM <= 0 || !Double.isFinite(wheelBaseM) || wheelBaseM <= 0)
            throw new IllegalArgumentException("Wheel radius, track width, and wheelbase must be positive");
        MecanumKinematics kinematics = new MecanumKinematics(trackWidthM, wheelBaseM, 2);
        String[] names = {"left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive"};
        List<DrivePair> pairs = new ArrayList<>();
        double maxTargetSpeed = 0, maxTargetYaw = 0;
        for (int i = 0; i + 1 < rows.size(); i++) {
            TelemetryLogRow cur = rows.get(i), next = rows.get(i + 1);
            if (cur.vxMps == null || next.vxMps == null) continue;
            double[] speed = new double[4];
            for (int w = 0; w < 4; w++) {
                MotorSpec spec = specs.get(names[w]);
                TelemetryLogRow.MotorSample sample = cur.motors.get(names[w]);
                if (spec == null || sample == null)
                    throw new IllegalArgumentException("Drive fit requires four named drive motors with preset specs");
                speed[w] = sample.velTps / spec.encoderCountsPerRev * 2 * Math.PI * wheelRadiusM;
            }
            var target = kinematics.forwardFromWheelSpeeds(speed[0], speed[1], speed[2], speed[3]);
            double dt = (next.tMs - cur.tMs) / 1000.0;
            if (dt <= 0 || dt > .2) continue;
            pairs.add(new DrivePair(dt, cur.vxMps, cur.vyMps, cur.omegaRadS, target.vx, target.vy,
                target.omega, next.vxMps, next.vyMps, next.omegaRadS));
            maxTargetSpeed = Math.max(maxTargetSpeed, Math.hypot(target.vx, target.vy));
            maxTargetYaw = Math.max(maxTargetYaw, Math.abs(target.omega));
        }
        if (pairs.size() < 30 || maxTargetSpeed < .3 || maxTargetYaw < .5)
            throw new IllegalArgumentException("Drive fit needs at least 30 paired chassis samples with forward/strafe and turning excitation");
        double bestTranslation = Double.POSITIVE_INFINITY, bestTau = 0, bestAccel = 0;
        double bestYaw = Double.POSITIVE_INFINITY, bestYawTau = 0, bestYawAccel = 0;
        for (int ti = 4; ti <= 40; ti++) {
            double tau = ti / 100.0;
            for (int ai = 4; ai <= 60; ai++) {
                double accel = ai / 4.0;
                double error = 0;
                for (DrivePair p : pairs) {
                    double response = -Math.expm1(-p.dt / tau);
                    double dx = (p.targetVx - p.vx) * response, dy = (p.targetVy - p.vy) * response;
                    double length = Math.hypot(dx, dy), cap = accel * p.dt;
                    if (length > cap) { dx *= cap / length; dy *= cap / length; }
                    error += square(p.vx + dx - p.nextVx) + square(p.vy + dy - p.nextVy);
                }
                if (error < bestTranslation) { bestTranslation = error; bestTau = tau; bestAccel = accel; }
            }
            for (int ai = 2; ai <= 80; ai++) {
                double accel = ai * .5;
                double error = 0;
                for (DrivePair p : pairs) {
                    double delta = (p.targetOmega - p.omega) * -Math.expm1(-p.dt / tau);
                    delta = Math.max(-accel * p.dt, Math.min(accel * p.dt, delta));
                    error += square(p.omega + delta - p.nextOmega);
                }
                if (error < bestYaw) { bestYaw = error; bestYawTau = tau; bestYawAccel = accel; }
            }
        }
        int cappedTranslation = 0, cappedYaw = 0;
        for (DrivePair p : pairs) {
            if (Math.hypot(p.targetVx - p.vx, p.targetVy - p.vy) * -Math.expm1(-p.dt / bestTau)
                > bestAccel * p.dt * 1.01) cappedTranslation++;
            if (Math.abs(p.targetOmega - p.omega) * -Math.expm1(-p.dt / bestYawTau)
                > bestYawAccel * p.dt * 1.01) cappedYaw++;
        }
        if (cappedTranslation < 3 || cappedYaw < 3)
            throw new IllegalArgumentException("Drive fit needs repeated acceleration-limited steps to identify traction caps");
        return new DriveFit(bestTau, bestAccel, bestYawTau, bestYawAccel,
            Math.sqrt(bestTranslation / (2 * pairs.size())), Math.sqrt(bestYaw / pairs.size()), pairs.size());
    }

    private static double square(double value) { return value * value; }
}
