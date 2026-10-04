package simcore;

/** Implemented by simulated odometry devices; the renderer feeds them the chassis pose each tick. */
public interface PoseSink {
    void onChassisPose(double worldX, double worldY, double worldYaw,
                       double worldVx, double worldVy, double yawRate);
}
