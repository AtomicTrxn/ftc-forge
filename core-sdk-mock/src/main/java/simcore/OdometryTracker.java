package simcore;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Random;

/**
 * Converts world-frame chassis samples into the pose an odometry computer reports: x forward,
 * y left, counterclockwise heading, relative to the pose where tracking started (or the pose
 * the team set). Shared by the Pinpoint and OTOS stubs. Units here are meters and radians.
 * The error model (latency, scale error, drift, noise) comes from {@link OdometryParams}.
 */
public class OdometryTracker {
    public record Pose(double x, double y, double heading) { }

    private record Sample(long timeMs, double x, double y, double yaw, double vx, double vy, double w,
                          double noiseX, double noiseY, double noiseH) { }

    private OdometryParams params = OdometryParams.IDEAL;
    private Random random = new Random(0);
    private final Deque<Sample> history = new ArrayDeque<>();
    private Pose base = new Pose(0, 0, 0);
    private Sample origin;            // sample the current tracking frame started at; null until the first sample
    private double unwrappedYaw;
    private boolean haveYaw;
    private Sample lastRaw;

    public synchronized void configure(OdometryParams params, String name) {
        this.params = params;
        this.random = new Random(params.seed * 31 + name.hashCode());
    }

    /** World-frame position/yaw plus world-frame linear velocity and yaw rate; {@code timeMs < 0} means no clock. */
    public synchronized void sample(long timeMs, double worldX, double worldY, double worldYaw,
                                    double worldVx, double worldVy, double yawRate) {
        if (!haveYaw) {
            unwrappedYaw = worldYaw;
            haveYaw = true;
        } else {
            double previousWrapped = Math.atan2(Math.sin(unwrappedYaw), Math.cos(unwrappedYaw));
            double step = Math.atan2(Math.sin(worldYaw - previousWrapped), Math.cos(worldYaw - previousWrapped));
            unwrappedYaw += step;
        }
        Sample s = new Sample(timeMs, worldX, worldY, unwrappedYaw, worldVx, worldVy, yawRate,
            gaussian(params.positionNoiseM), gaussian(params.positionNoiseM), gaussian(params.headingNoiseRad));
        if (!history.isEmpty() && timeMs >= 0 && history.peekLast().timeMs > timeMs) history.clear(); // clock restarted
        history.addLast(s);
        while (history.size() > 2 && timeMs >= 0 && history.stream().skip(1).findFirst().get().timeMs < timeMs - 1000)
            history.pollFirst();
        if (history.size() > 4096) history.pollFirst();
        lastRaw = s;
        if (origin == null) origin = view();
    }

    private double gaussian(double std) { return std == 0 ? 0 : random.nextGaussian() * std; }

    /** The sample the device would report now: the latest, or one interpolated {@code latencyMs} ago. */
    private Sample view() {
        if (lastRaw == null) return null;
        if (params.latencyMs == 0 || lastRaw.timeMs < 0) return lastRaw;
        long target = lastRaw.timeMs - params.latencyMs;
        Sample previous = null;
        for (Sample s : history) {
            if (s.timeMs > target) {
                if (previous == null) return history.peekFirst();
                double f = (double) (target - previous.timeMs) / (s.timeMs - previous.timeMs);
                return new Sample(target, lerp(previous.x, s.x, f), lerp(previous.y, s.y, f), lerp(previous.yaw, s.yaw, f),
                    lerp(previous.vx, s.vx, f), lerp(previous.vy, s.vy, f), lerp(previous.w, s.w, f),
                    lerp(previous.noiseX, s.noiseX, f), lerp(previous.noiseY, s.noiseY, f), lerp(previous.noiseH, s.noiseH, f));
            }
            previous = s;
        }
        return previous;
    }

    private static double lerp(double a, double b, double f) { return a + (b - a) * f; }

    public synchronized Pose pose() {
        Sample now = view();
        if (now == null || origin == null) return base;
        double dx = now.x - origin.x, dy = now.y - origin.y;
        double c = Math.cos(origin.yaw), s = Math.sin(origin.yaw);
        double lx = (c * dx + s * dy) * (1 + params.linearScaleError) + now.noiseX;
        double ly = (-s * dx + c * dy) * (1 + params.linearScaleError) + now.noiseY;
        double elapsedS = now.timeMs >= 0 && origin.timeMs >= 0 ? (now.timeMs - origin.timeMs) / 1000.0 : 0;
        double dh = (now.yaw - origin.yaw) * (1 + params.headingScaleError) + params.headingDriftRadS * elapsedS + now.noiseH;
        double cb = Math.cos(base.heading), sb = Math.sin(base.heading);
        return new Pose(base.x + cb * lx - sb * ly, base.y + sb * lx + cb * ly, base.heading + dh);
    }

    public synchronized Pose velocity() {
        Sample now = view();
        if (now == null || origin == null) return new Pose(0, 0, 0);
        double c = Math.cos(origin.yaw), s = Math.sin(origin.yaw);
        double vx = c * now.vx + s * now.vy, vy = -s * now.vx + c * now.vy;
        double cb = Math.cos(base.heading), sb = Math.sin(base.heading);
        double scale = 1 + params.linearScaleError;
        return new Pose((cb * vx - sb * vy) * scale, (sb * vx + cb * vy) * scale,
            now.w * (1 + params.headingScaleError));
    }

    /** Reports {@code pose} at the current (as measured) position, like setPosition on real hardware. */
    public synchronized void set(Pose pose) {
        base = pose;
        origin = view();
    }

    public synchronized void reset() { set(new Pose(0, 0, 0)); }
}
