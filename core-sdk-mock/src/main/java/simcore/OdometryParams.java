package simcore;

/**
 * Tunable error model for odometry computers (Pinpoint, OTOS). Defaults are an ideal sensor.
 * Every field is meant to be set from {@code sim.config} and later fitted from a real recording;
 * none are measured values.
 */
public final class OdometryParams {
    /** Delay between the chassis moving and the device reporting it. 0 returns the latest pose. */
    public final long latencyMs;
    /** Fractional error on reported distance (0.01 reads 1% long), e.g. a wrong ticks-per-mm constant. */
    public final double linearScaleError;
    /** Fractional error on reported heading change (e.g. a wrong yaw scalar). */
    public final double headingScaleError;
    /** Constant heading drift, radians per second since tracking started or was reset. */
    public final double headingDriftRadS;
    /** Standard deviation of per-update position noise in meters. */
    public final double positionNoiseM;
    /** Standard deviation of per-update heading noise in radians. */
    public final double headingNoiseRad;
    /** Seed so noisy runs repeat; each device mixes in its own name. */
    public final long seed;
    /** Time each pose read or update blocks the caller, modelling the I2C round trip. */
    public final double readCostMs;

    public static final OdometryParams IDEAL = new OdometryParams(0, 0, 0, 0, 0, 0, 0, 0);

    public OdometryParams(long latencyMs, double linearScaleError, double headingScaleError,
                          double headingDriftRadS, double positionNoiseM, double headingNoiseRad, long seed) {
        this(latencyMs, linearScaleError, headingScaleError, headingDriftRadS, positionNoiseM, headingNoiseRad, seed, 0);
    }

    public OdometryParams(long latencyMs, double linearScaleError, double headingScaleError,
                          double headingDriftRadS, double positionNoiseM, double headingNoiseRad, long seed, double readCostMs) {
        if (latencyMs < 0 || latencyMs > 200) throw new IllegalArgumentException("odometry latency_ms must be 0..200");
        if (!(Math.abs(linearScaleError) <= 0.5)) throw new IllegalArgumentException("linear_scale_error must be within +/-0.5");
        if (!(Math.abs(headingScaleError) <= 0.5)) throw new IllegalArgumentException("heading_scale_error must be within +/-0.5");
        if (!(Math.abs(headingDriftRadS) <= 0.1)) throw new IllegalArgumentException("heading_drift_rad_s must be within +/-0.1");
        if (!(positionNoiseM >= 0 && positionNoiseM <= 0.05)) throw new IllegalArgumentException("position_noise_m must be 0..0.05");
        if (!(headingNoiseRad >= 0 && headingNoiseRad <= 0.1)) throw new IllegalArgumentException("heading_noise_rad must be 0..0.1");
        if (!(readCostMs >= 0 && readCostMs <= 50)) throw new IllegalArgumentException("odometry read_cost_ms must be 0..50");
        this.readCostMs = readCostMs;
        this.latencyMs = latencyMs;
        this.linearScaleError = linearScaleError;
        this.headingScaleError = headingScaleError;
        this.headingDriftRadS = headingDriftRadS;
        this.positionNoiseM = positionNoiseM;
        this.headingNoiseRad = headingNoiseRad;
        this.seed = seed;
    }
}
