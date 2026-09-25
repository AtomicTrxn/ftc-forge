package physics;

/** Integrates chassis-frame velocity into a field-frame pose. Ideal (no-slip) integration -- real drift/slip is out of scope until Phase 4/5. */
public class ChassisPose {
    public double xMeters;
    public double yMeters;
    public double headingRad;

    public void integrate(MecanumKinematics.ChassisVelocity v, double dtSeconds) {
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double fieldVx = v.vx * cos - v.vy * sin;
        double fieldVy = v.vx * sin + v.vy * cos;
        xMeters += fieldVx * dtSeconds;
        yMeters += fieldVy * dtSeconds;
        headingRad += v.omega * dtSeconds;
    }

    @Override public String toString() {
        return String.format("ChassisPose(x=%.4fm, y=%.4fm, heading=%.2fdeg)", xMeters, yMeters, Math.toDegrees(headingRad));
    }
}
