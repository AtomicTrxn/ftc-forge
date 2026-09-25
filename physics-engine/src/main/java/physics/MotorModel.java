package physics;

/**
 * Motor non-ideality model per R4's corrected equations:
 *
 *   tau = tauStall * (Vactual/Vnominal - omega/omegaNoLoad)
 *   I   = iStall    * (Vactual/Vnominal - omega/omegaNoLoad)
 *
 * ADDITION beyond R4 (documented, not silently assumed to already be in R4's research):
 * R4's equations are the ideal EMF-based torque/current relation and have no free
 * parameter for real motors' mechanical friction losses. This class adds a standard
 * static+viscous friction term (tauStatic, viscousB) -- exactly the kind of thing Road
 * Runner/Pedro call kS/kV feedforward terms -- as calibration targets for R5's staged
 * fit, since R4's own torque-curve constants (tauStall, omegaNoLoad, iStall) are fixed,
 * sourced datasheet values, not tuning targets.
 *
 * Thermal throttling (R4: "no vendor publishes a curve, model as a duty-cycle heuristic")
 * is implemented as a simple rolling-heat state, present but not exercised by this
 * phase's calibration pass unless a long, sustained-load run actually needs it (per R5's
 * own staged-fit design: "thermal only if long-run residuals justify it").
 */
public class MotorModel {

    public double tauStaticNm = 0.0;   // Coulomb friction torque, opposing motion
    public double viscousBNms = 0.0;   // viscous friction coefficient (N*m*s/rad)

    // Thermal state (simple placeholder heuristic, per R4).
    private double heat = 0.0;
    public double thermalThresholdAmps = 0.7; // fraction of stall current above which heat accumulates
    public double thermalTimeConstantS = 20.0; // seconds of sustained overload before meaningful derating
    public double thermalMaxDerate = 0.3;      // max fractional torque reduction at full heat

    public double torque(MotorSpec spec, double omega, double vActual) {
        double idealTorque = spec.tauStallNm * (vActual / spec.vNominal - omega / spec.omegaNoLoadRadS);
        double friction = Math.signum(omega) * tauStaticNm + viscousBNms * omega;
        double derate = 1.0 - thermalMaxDerate * thermalFraction();
        return idealTorque * derate - friction;
    }

    public double current(MotorSpec spec, double omega, double vActual) {
        return spec.iStallAmps * (vActual / spec.vNominal - omega / spec.omegaNoLoadRadS);
    }

    /** Advances the thermal heat state by one tick, given the current draw magnitude. */
    public void tickThermal(MotorSpec spec, double currentAmps, double dtSeconds) {
        double overloadFraction = Math.abs(currentAmps) / spec.iStallAmps;
        double target = overloadFraction > thermalThresholdAmps ? 1.0 : 0.0;
        double rate = dtSeconds / thermalTimeConstantS;
        heat += (target - heat) * rate;
        heat = Math.max(0.0, Math.min(1.0, heat));
    }

    public double thermalFraction() {
        return heat;
    }
}
