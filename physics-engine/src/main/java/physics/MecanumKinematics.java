package physics;

/**
 * Ideal (no-slip) Mecanum wheel kinematics, per Phase 2's spec.
 *
 * Deliberately engine-agnostic: forward() takes wheel powers and returns a chassis-frame
 * ChassisVelocity (vx, vy, omega), not a 2D-only position update. Phase 4 applies this same
 * output as a force/torque directly to a Libbulletjme rigid body (per R2/R6's finding that
 * wheel-ground contact physics cannot itself produce Mecanum strafing) -- this class doesn't
 * change between Phase 2 and Phase 4, only what consumes its output does.
 *
 * Wheel layout convention: standard "X" Mecanum arrangement, rollers at +/-45 degrees,
 * viewed from above with +x forward, +y left, +omega counter-clockwise:
 *
 *   LF \\_/ RF
 *       | |
 *   LB /_\\ RB
 */
public class MecanumKinematics {

    public final double maxWheelSpeedMetersPerSecond;
    private final double turnRadius; // (trackWidth + wheelBase) / 2

    public MecanumKinematics(double trackWidthMeters, double wheelBaseMeters, double maxWheelSpeedMetersPerSecond) {
        this.turnRadius = (trackWidthMeters + wheelBaseMeters) / 2.0;
        this.maxWheelSpeedMetersPerSecond = maxWheelSpeedMetersPerSecond;
    }

    public static class ChassisVelocity {
        public final double vx;    // m/s, +forward
        public final double vy;    // m/s, +left
        public final double omega; // rad/s, +counter-clockwise

        public ChassisVelocity(double vx, double vy, double omega) {
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
        }

        @Override public String toString() {
            return String.format("ChassisVelocity(vx=%.4f, vy=%.4f, omega=%.4f)", vx, vy, omega);
        }
    }

    /** Forward kinematics: four wheel powers in [-1, 1] -> chassis-frame velocity. */
    public ChassisVelocity forward(double powerLF, double powerRF, double powerLB, double powerRB) {
        double vLF = powerLF * maxWheelSpeedMetersPerSecond;
        double vRF = powerRF * maxWheelSpeedMetersPerSecond;
        double vLB = powerLB * maxWheelSpeedMetersPerSecond;
        double vRB = powerRB * maxWheelSpeedMetersPerSecond;

        double vx = (vLF + vRF + vLB + vRB) / 4.0;
        double vy = (-vLF + vRF + vLB - vRB) / 4.0;
        double omega = (-vLF + vRF - vLB + vRB) / (4.0 * turnRadius);

        return new ChassisVelocity(vx, vy, omega);
    }

    /** Inverse kinematics: desired chassis-frame velocity -> four wheel powers in [-1, 1], for manual/gamepad driving. */
    public double[] inverse(double vx, double vy, double omega) {
        double vLF = vx - vy - omega * turnRadius;
        double vRF = vx + vy + omega * turnRadius;
        double vLB = vx + vy - omega * turnRadius;
        double vRB = vx - vy + omega * turnRadius;

        double maxAbs = Math.max(Math.max(Math.abs(vLF), Math.abs(vRF)), Math.max(Math.abs(vLB), Math.abs(vRB)));
        double scale = maxAbs > maxWheelSpeedMetersPerSecond ? maxWheelSpeedMetersPerSecond / maxAbs : 1.0;

        return new double[] {
            (vLF * scale) / maxWheelSpeedMetersPerSecond,
            (vRF * scale) / maxWheelSpeedMetersPerSecond,
            (vLB * scale) / maxWheelSpeedMetersPerSecond,
            (vRB * scale) / maxWheelSpeedMetersPerSecond
        };
    }
}
