package simcore;

/**
 * Optional overrides for one motor's placeholder model parameters. A null field leaves the
 * built-in default; a calibration profile applied later still replaces the friction values.
 * All values are starting estimates meant to be tuned, not measurements.
 */
public record MotorTuning(Double rotorInertiaKgM2, Double staticFrictionNm, Double viscousFrictionNmSPerRad,
                          Double thermalThresholdFraction, Double thermalTimeConstantS, Double thermalMaxDerate,
                          Double velocityPGain, Double positionPGain,
                          Double windingCapacityJPerK, Double windingResistanceKPerW, Double ambientC, Double derateStartC,
                          Double derateFullC, Double windingResistanceOhm, Double resistanceTempcoPerK) {
    public static final MotorTuning NONE = new MotorTuning(null, null, null, null, null, null, null, null,
        null, null, null, null, null, null, null);

    /** The original eight-field form (no winding thermal model). */
    public MotorTuning(Double rotorInertiaKgM2, Double staticFrictionNm, Double viscousFrictionNmSPerRad,
                       Double thermalThresholdFraction, Double thermalTimeConstantS, Double thermalMaxDerate,
                       Double velocityPGain, Double positionPGain) {
        this(rotorInertiaKgM2, staticFrictionNm, viscousFrictionNmSPerRad, thermalThresholdFraction, thermalTimeConstantS,
            thermalMaxDerate, velocityPGain, positionPGain, null, null, null, null, null, null, null);
    }

    public MotorTuning {
        positive(rotorInertiaKgM2, "rotor_inertia_kg_m2", 1e-7, 10);
        range(staticFrictionNm, "static_friction_nm", 0, 20);
        range(viscousFrictionNmSPerRad, "viscous_friction_nm_s_per_rad", 0, 20);
        range(thermalThresholdFraction, "thermal_threshold_fraction", 0, 1);
        positive(thermalTimeConstantS, "thermal_time_constant_s", 0.01, 36000);
        range(thermalMaxDerate, "thermal_max_derate", 0, 1);
        range(velocityPGain, "velocity_p_gain", 0, 1000);
        range(positionPGain, "position_p_gain", 0, 1000);
        range(windingCapacityJPerK, "thermal_capacity_j_per_k", 0, 100000);
        range(windingResistanceKPerW, "thermal_resistance_k_per_w", 0.01, 1000);
        range(ambientC, "ambient_c", -40, 80);
        range(derateStartC, "derate_start_c", -40, 400);
        range(derateFullC, "derate_full_c", -40, 400);
        range(windingResistanceOhm, "winding_resistance_ohm", 0, 100);
        range(resistanceTempcoPerK, "resistance_tempco_per_k", 0, 0.05);
        if (derateStartC != null && derateFullC != null && derateFullC <= derateStartC)
            throw new IllegalArgumentException("derate_full_c must exceed derate_start_c");
    }

    /** This tuning, with any field it leaves null taken from {@code fallback}. */
    public MotorTuning over(MotorTuning fallback) {
        return new MotorTuning(pick(rotorInertiaKgM2, fallback.rotorInertiaKgM2), pick(staticFrictionNm, fallback.staticFrictionNm),
            pick(viscousFrictionNmSPerRad, fallback.viscousFrictionNmSPerRad), pick(thermalThresholdFraction, fallback.thermalThresholdFraction),
            pick(thermalTimeConstantS, fallback.thermalTimeConstantS), pick(thermalMaxDerate, fallback.thermalMaxDerate),
            pick(velocityPGain, fallback.velocityPGain), pick(positionPGain, fallback.positionPGain),
            pick(windingCapacityJPerK, fallback.windingCapacityJPerK), pick(windingResistanceKPerW, fallback.windingResistanceKPerW),
            pick(ambientC, fallback.ambientC), pick(derateStartC, fallback.derateStartC), pick(derateFullC, fallback.derateFullC),
            pick(windingResistanceOhm, fallback.windingResistanceOhm), pick(resistanceTempcoPerK, fallback.resistanceTempcoPerK));
    }

    private static Double pick(Double mine, Double theirs) { return mine != null ? mine : theirs; }

    private static void range(Double v, String name, double low, double high) {
        if (v != null && !(v >= low && v <= high)) throw new IllegalArgumentException(name + " must be " + low + ".." + high);
    }

    private static void positive(Double v, String name, double low, double high) { range(v, name, low, high); }
}
