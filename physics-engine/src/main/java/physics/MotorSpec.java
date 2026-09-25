package physics;

/** Per-SKU/ratio motor datasheet constants (per R3/R4: the real robot-config XML can't carry these, so presets do). */
public class MotorSpec {
    public final String sku;
    public final double ratio;
    public final double tauStallNm;      // stall torque at V_nominal, at the geared output shaft
    public final double iStallAmps;      // stall current at V_nominal
    public final double omegaNoLoadRadS; // no-load speed at V_nominal, at the geared output shaft
    public final double vNominal;        // nominal voltage the above are specified at (12V for goBILDA/REV)
    public final double encoderCountsPerRev; // at the geared output shaft

    public MotorSpec(String sku, double ratio, double tauStallNm, double iStallAmps,
                      double omegaNoLoadRadS, double vNominal, double encoderCountsPerRev) {
        this.sku = sku;
        this.ratio = ratio;
        this.tauStallNm = tauStallNm;
        this.iStallAmps = iStallAmps;
        this.omegaNoLoadRadS = omegaNoLoadRadS;
        this.vNominal = vNominal;
        this.encoderCountsPerRev = encoderCountsPerRev;
    }
}
