package simcore;

/** Implemented by simulated odometry devices; the renderer feeds them the chassis pose each tick. */
public interface PoseSink {
    /** {@code simTimeMs} is the simulator clock, or negative when the caller has none (latency and drift are then skipped). */
    void onChassisPose(long simTimeMs, double worldX, double worldY, double worldYaw,
                       double worldVx, double worldVy, double yawRate);

    default void onChassisPose(double worldX, double worldY, double worldYaw,
                               double worldVx, double worldVy, double yawRate) {
        onChassisPose(-1, worldX, worldY, worldYaw, worldVx, worldVy, yawRate);
    }

    /** Sets the error model; called after the hardware map is built. */
    void configureOdometry(OdometryParams params, String deviceName);
}
