package simcore;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import physics.SensorRingBuffer;

/** IMU orientation and angular rate sampled from the physics chassis on the simulator clock. */
public class SimIMU implements IMU {
    private record Sample(double yawRad,double pitchRad,double rollRad,double omegaX,double omegaY,double omegaZ,long timeMs) { }

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
        update(yawRad,0,0,0,0,yawRateRadS,simTimeMs);
    }
    /** Intrinsic ZYX orientation and angular rates in the configured robot frame, radians. */
    public synchronized void update(double yawRad,double pitchRad,double rollRad,double omegaX,double omegaY,double omegaZ,long simTimeMs) {
        for(double value:new double[]{yawRad,pitchRad,rollRad,omegaX,omegaY,omegaZ})if(!Double.isFinite(value))throw new IllegalArgumentException("IMU pose/rate must be finite");
        if (simTimeMs < this.simTimeMs) throw new IllegalArgumentException("IMU simulation time moved backwards");
        this.simTimeMs = simTimeMs;
        samples.push(simTimeMs, new Sample(yawRad,pitchRad,rollRad,omegaX,omegaY,omegaZ,simTimeMs));
    }

    private Sample observed() {
        Sample delayed = samples.read(simTimeMs, latencyMs);
        return delayed != null ? delayed : new Sample(0,0,0,0,0,0,0);
    }

    @Override public synchronized boolean initialize(Parameters parameters) {
        this.parameters = parameters;
        return true;
    }

    @Override public synchronized YawPitchRollAngles getRobotYawPitchRollAngles() {
        Sample sample = observed();
        double yaw = Math.atan2(Math.sin(sample.yawRad - yawZeroRad), Math.cos(sample.yawRad - yawZeroRad));
        return new YawPitchRollAngles(Math.toDegrees(yaw),Math.toDegrees(sample.pitchRad),Math.toDegrees(sample.rollRad),sample.timeMs * 1_000_000L);
    }

    @Override public synchronized AngularVelocity getRobotAngularVelocity(AngleUnit unit) {
        Sample s=observed();double scale=unit==AngleUnit.DEGREES?180/Math.PI:1;
        return new AngularVelocity(unit,s.omegaX*scale,s.omegaY*scale,s.omegaZ*scale);
    }

    @Override public synchronized void resetYaw() { yawZeroRad = observed().yawRad; }

    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated IMU \"" + name + "\""; }
    @Override public void close() { }
}
