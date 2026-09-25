package physics;

import java.util.List;

/**
 * Closed-form multi-motor battery sag model, exactly per R4's derivation:
 *
 *   A = sum(p_i^2 * iStall_i / vNominal_i)
 *   B = sum(p_i * iStall_i * (omega_i / omegaNoLoad_i))
 *   Vb = (Vinternal + Rbattery * B) / (1 + Rbattery * A)
 *
 * Duty-averaged H-bridge approximation (R4): does not model per-PWM-cycle ripple,
 * appropriate for a per-tick simulation running far slower than PWM switching frequency.
 */
public class BatteryModel {

    public double vInternal;
    public double rBattery;

    public BatteryModel(double vInternal, double rBattery) {
        this.vInternal = vInternal;
        this.rBattery = rBattery;
    }

    public static class MotorState {
        public final double power;   // commanded power [-1, 1], i.e. duty cycle
        public final MotorSpec spec;
        public final double omega;   // current angular velocity, rad/s

        public MotorState(double power, MotorSpec spec, double omega) {
            this.power = power;
            this.spec = spec;
            this.omega = omega;
        }
    }

    public double solveBatteryVoltage(List<MotorState> motors) {
        double a = 0.0;
        double b = 0.0;
        for (MotorState m : motors) {
            a += m.power * m.power * m.spec.iStallAmps / m.spec.vNominal;
            b += m.power * m.spec.iStallAmps * (m.omega / m.spec.omegaNoLoadRadS);
        }
        return (vInternal + rBattery * b) / (1 + rBattery * a);
    }
}
