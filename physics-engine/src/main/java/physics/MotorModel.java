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

    // Winding thermal model, used instead of the heuristic above when thermalCapacityJPerK > 0. Copper heats from
    // I^2*R, loses heat to the ambient through a thermal resistance, and a hotter winding has higher resistance
    // (less torque and current for the same voltage). Every value is a placeholder, not a vendor figure.
    /** Winding heat capacity, J/K; 0 keeps the duty-cycle heuristic. */
    public double thermalCapacityJPerK = 0.0;
    /** Winding-to-ambient thermal resistance, K/W. */
    public double thermalResistanceKPerW = 6.0;
    public double ambientC = 25.0;
    /** Winding temperature where torque derating begins and where it reaches {@link #thermalMaxDerate}. */
    public double derateStartC = 80.0;
    public double derateFullC = 130.0;
    /** Winding resistance at 25 C; 0 derives it from the datasheet as vNominal / iStall. */
    public double windingResistanceOhm = 0.0;
    /** Fractional resistance rise per kelvin (copper is about 0.0039). */
    public double resistanceTempcoPerK = 0.0039;
    private double windingTempC = Double.NaN;

    public boolean windingModelEnabled() { return thermalCapacityJPerK > 0; }

    public double windingTempC() {
        return Double.isNaN(windingTempC) ? ambientC : windingTempC;
    }

    public void resetThermal() { heat = 0; windingTempC = Double.NaN; }

    private double windingResistance(MotorSpec spec) {
        return windingResistanceOhm > 0 ? windingResistanceOhm : spec.vNominal / spec.iStallAmps;
    }

    /** Multiplier on torque and current from winding resistance rising above 25 C (1 when the winding model is off). */
    private double resistanceScale() {
        if (!windingModelEnabled()) return 1.0;
        return 1.0 / (1.0 + resistanceTempcoPerK * (windingTempC() - 25.0));
    }

    private double derate() {
        if (windingModelEnabled()) {
            double span = Math.max(1e-9, derateFullC - derateStartC);
            double fraction = Math.max(0.0, Math.min(1.0, (windingTempC() - derateStartC) / span));
            return 1.0 - thermalMaxDerate * fraction;
        }
        return 1.0 - thermalMaxDerate * thermalFraction();
    }

    public double torque(MotorSpec spec, double omega, double vActual) {
        double idealTorque = spec.tauStallNm * (vActual / spec.vNominal - omega / spec.omegaNoLoadRadS) * resistanceScale();
        double friction = Math.signum(omega) * tauStaticNm + viscousBNms * omega;
        return idealTorque * derate() - friction;
    }

    /** Mechanical friction opposing motion (the same term {@link #torque} subtracts). */
    public double frictionTorque(double omega) {
        return Math.signum(omega) * tauStaticNm + viscousBNms * omega;
    }

    public double current(MotorSpec spec, double omega, double vActual) {
        return spec.iStallAmps * (vActual / spec.vNominal - omega / spec.omegaNoLoadRadS) * resistanceScale();
    }

    /** Advances the thermal heat state by one tick, given the current draw magnitude. */
    public void tickThermal(MotorSpec spec, double currentAmps, double dtSeconds) {
        if (windingModelEnabled()) {
            double temperature = windingTempC();
            double resistance = windingResistance(spec) * (1.0 + resistanceTempcoPerK * (temperature - 25.0));
            double steadyC = ambientC + currentAmps * currentAmps * resistance * thermalResistanceKPerW;
            double timeConstantS = thermalResistanceKPerW * thermalCapacityJPerK;
            // Exact first-order step toward the steady temperature for this heat input: stable at any tick length.
            windingTempC = steadyC + (temperature - steadyC) * Math.exp(-dtSeconds / timeConstantS);
            return;
        }
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
