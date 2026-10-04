package simcore;

/**
 * Converts world-frame chassis samples into the pose an odometry computer reports: x forward,
 * y left, heading counterclockwise, relative to the pose where tracking started (or the pose
 * the team set). Shared by the Pinpoint and OTOS stubs. Units here are meters and radians.
 */
public class OdometryTracker {
    public record Pose(double x, double y, double heading) { }

    private Pose base = new Pose(0, 0, 0);
    private Pose origin;      // world pose at the last reset; null until the first sample
    private Pose world;       // latest world sample
    private Pose velocity = new Pose(0, 0, 0);

    /** World-frame position/yaw plus world-frame linear velocity and yaw rate. */
    public synchronized void sample(double worldX, double worldY, double worldYaw,
                                    double worldVx, double worldVy, double yawRate) {
        world = new Pose(worldX, worldY, worldYaw);
        if (origin == null) origin = world;
        double c = Math.cos(origin.heading), s = Math.sin(origin.heading);
        double vx = c * worldVx + s * worldVy;
        double vy = -s * worldVx + c * worldVy;
        double cb = Math.cos(base.heading), sb = Math.sin(base.heading);
        velocity = new Pose(cb * vx - sb * vy, sb * vx + cb * vy, yawRate);
    }

    public synchronized Pose pose() {
        if (origin == null) return base;
        double dx = world.x - origin.x, dy = world.y - origin.y;
        double c = Math.cos(origin.heading), s = Math.sin(origin.heading);
        double lx = c * dx + s * dy, ly = -s * dx + c * dy;
        double dh = world.heading - origin.heading;
        double cb = Math.cos(base.heading), sb = Math.sin(base.heading);
        return new Pose(base.x + cb * lx - sb * ly, base.y + sb * lx + cb * ly, base.heading + dh);
    }

    public synchronized Pose velocity() { return velocity; }

    /** Reports {@code pose} at the current physical position (like setPosition on real hardware). */
    public synchronized void set(Pose pose) {
        base = pose;
        origin = world;
    }

    public synchronized void reset() { set(new Pose(0, 0, 0)); }
}
