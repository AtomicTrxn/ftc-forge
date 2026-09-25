package physics.calibration;

import physics.MotorModel;
import physics.MotorSpec;

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
     * Itotal here is the duty-scaled battery-rail current (power * current per motor, summed),
     * matching BatteryModel's own closed-form derivation exactly -- not the raw per-motor
     * current alone. Using raw |current| instead systematically biases the fit whenever any
     * motor runs below full power, since a real battery only sees a duty-cycle-averaged share
     * of each motor's coil current, not the full instantaneous value (this was caught by a
     * failing recovery test against a synthetic ground-truth log, not assumed correct).
     */
    public static BatteryFit fitBattery(List<TelemetryLogRow> rows) {
        int n = rows.size();
        double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
        for (TelemetryLogRow row : rows) {
            double iTotal = 0;
            for (TelemetryLogRow.MotorSample s : row.motors.values()) iTotal += s.power * s.currentA;
            double v = row.batteryVoltageV;
            sumX += iTotal;
            sumY += v;
            sumXY += iTotal * v;
            sumXX += iTotal * iTotal;
        }
        double slope = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX); // slope = -Rbattery
        double intercept = (sumY - slope * sumX) / n;                        // intercept = Vinternal
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
                                                double rotationalInertiaKgM2, double batteryVInternal, double rBattery) {
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
        return new FrictionFit(bestTauStatic, bestViscousB, bestError);
    }
}
