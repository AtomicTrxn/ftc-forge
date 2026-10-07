package simcore;

/**
 * Tunable IMU error model. Defaults are an ideal sensor. Every field is meant to be set from
 * {@code sim.config} and later fitted from a recording; none are measured values.
 */
public final class ImuParams {
    /** Fractional error on reported yaw change (e.g. 0.01 reads 1% too much rotation). */
    public final double yawScaleError;
    /** Constant yaw drift in radians per second since the last {@code resetYaw()} (or start). */
    public final double yawDriftRadS;
    /** Constant gyro rate bias added to every axis, rad/s. */
    public final double gyroBiasRadS;
    /** Standard deviation of per-sample gyro rate noise, rad/s. */
    public final double gyroNoiseRadS;
    /** Standard deviation of per-sample orientation noise, rad. */
    public final double angleNoiseRad;
    /** Seed so noisy runs repeat; each device mixes in its own name. */
    public final long seed;
    /** Time each orientation or rate read blocks the caller, modelling the I2C round trip. */
    public final double readCostMs;

    public static final ImuParams IDEAL = new ImuParams(0, 0, 0, 0, 0, 0, 0);

    public ImuParams(double yawScaleError, double yawDriftRadS, double gyroBiasRadS, double gyroNoiseRadS,
                     double angleNoiseRad, long seed, double readCostMs) {
        if (!(Math.abs(yawScaleError) <= 0.5)) throw new IllegalArgumentException("yaw_scale_error must be within +/-0.5");
        if (!(Math.abs(yawDriftRadS) <= 0.1)) throw new IllegalArgumentException("yaw_drift_rad_s must be within +/-0.1");
        if (!(Math.abs(gyroBiasRadS) <= 0.5)) throw new IllegalArgumentException("gyro_bias_rad_s must be within +/-0.5");
        if (!(gyroNoiseRadS >= 0 && gyroNoiseRadS <= 0.5)) throw new IllegalArgumentException("gyro_noise_rad_s must be 0..0.5");
        if (!(angleNoiseRad >= 0 && angleNoiseRad <= 0.1)) throw new IllegalArgumentException("angle_noise_rad must be 0..0.1");
        if (!(readCostMs >= 0 && readCostMs <= 50)) throw new IllegalArgumentException("imu read_cost_ms must be 0..50");
        this.yawScaleError = yawScaleError;
        this.yawDriftRadS = yawDriftRadS;
        this.gyroBiasRadS = gyroBiasRadS;
        this.gyroNoiseRadS = gyroNoiseRadS;
        this.angleNoiseRad = angleNoiseRad;
        this.seed = seed;
        this.readCostMs = readCostMs;
    }
}
