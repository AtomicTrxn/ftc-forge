package physics;

/** Two powered sides, x forward, y left, positive yaw counterclockwise. */
public final class DifferentialKinematics {
    private final double trackWidthM;
    public DifferentialKinematics(double trackWidthM) {
        if (!Double.isFinite(trackWidthM) || trackWidthM <= 0)
            throw new IllegalArgumentException("Differential track width must be finite and positive");
        this.trackWidthM = trackWidthM;
    }
    public MecanumKinematics.ChassisVelocity forwardFromWheelSpeeds(double leftMps, double rightMps) {
        return new MecanumKinematics.ChassisVelocity((leftMps + rightMps) / 2, 0,
            (rightMps - leftMps) / trackWidthM);
    }
}
