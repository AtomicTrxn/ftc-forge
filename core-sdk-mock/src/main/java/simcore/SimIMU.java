package simcore;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import physics.SensorRingBuffer;

/** IMU yaw and yaw rate sampled from the physics chassis on the simulator clock. */
public class SimIMU implements IMU {
    private record Sample(double yawRad, double yawRateRadS, long timeMs) { }

    private final String name;
    private final SensorRingBuffer<Sample> samples = new SensorRingBuffer<>(1000);
    private Parameters parameters;
    private long latencyMs = 8;
    private long simTimeMs;
    private double yawZeroRad;

    public SimIMU(String name) { this.name = name; }

    public synchronized void setLatencyMs(long latencyMs) {
        if (latencyMs < 0 || latencyMs > 200) throw new IllegalArgumentException("IMU latency must be 0..200 ms");
        this.latencyMs = latencyMs;
    }

    /** Positive yaw turns the robot left about its vertical axis; pitch/roll are level. */
    public synchronized void update(double yawRad, double yawRateRadS, long simTimeMs) {
        if (simTimeMs < this.simTimeMs) throw new IllegalArgumentException("IMU simulation time moved backwards");
        this.simTimeMs = simTimeMs;
        samples.push(simTimeMs, new Sample(yawRad, yawRateRadS, simTimeMs));
    }

    private Sample observed() {
        Sample delayed = samples.read(simTimeMs, latencyMs);
        return delayed != null ? delayed : new Sample(0, 0, 0);
    }

    @Override public synchronized boolean initialize(Parameters parameters) {
        this.parameters = parameters;
        return true;
    }

    @Override public synchronized YawPitchRollAngles getRobotYawPitchRollAngles() {
        Sample sample = observed();
        double yaw = Math.atan2(Math.sin(sample.yawRad - yawZeroRad), Math.cos(sample.yawRad - yawZeroRad));
        return new YawPitchRollAngles(Math.toDegrees(yaw), 0, 0, sample.timeMs * 1_000_000L);
    }

    @Override public synchronized AngularVelocity getRobotAngularVelocity(AngleUnit unit) {
        double rate = observed().yawRateRadS;
        return new AngularVelocity(unit, 0, 0, unit == AngleUnit.DEGREES ? Math.toDegrees(rate) : rate);
    }

    @Override public synchronized void resetYaw() { yawZeroRad = observed().yawRad; }

    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated IMU \"" + name + "\""; }
    @Override public void close() { }
}
