package simcore;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import physics.SensorRingBuffer;

import java.util.Random;

/** IMU orientation and angular rate sampled from the physics chassis on the simulator clock. */
public class SimIMU implements IMU {
    private record Sample(double yawRad,double pitchRad,double rollRad,double omegaX,double omegaY,double omegaZ,long timeMs,
                          double noiseYaw,double noisePitch,double noiseRoll,double noiseX,double noiseY,double noiseZ) { }

    private final String name;
    private final SensorRingBuffer<Sample> samples = new SensorRingBuffer<>(1000);
    private Parameters parameters;
    private long latencyMs = 8;
    private long simTimeMs;
    private double yawZeroRad;
    private long yawZeroTimeMs;
    private ImuParams params = ImuParams.IDEAL;
    private Random random = new Random(0);

    public SimIMU(String name) { this.name = name; }

    public synchronized void setLatencyMs(long latencyMs) {
        if (latencyMs < 0 || latencyMs > 200) throw new IllegalArgumentException("IMU latency must be 0..200 ms");
        this.latencyMs = latencyMs;
    }

    public synchronized long getLatencyMs() { return latencyMs; }

    /** Sets the error model; call after the hardware map is built. */
    public synchronized void configureErrors(ImuParams params, String deviceName) {
        this.params = params;
        this.random = new Random(params.seed * 31 + deviceName.hashCode());
    }

    private double gaussian(double std) { return std == 0 ? 0 : random.nextGaussian() * std; }

    /** Positive yaw turns the robot left about its vertical axis; pitch/roll are level. */
    public synchronized void update(double yawRad, double yawRateRadS, long simTimeMs) {
        update(yawRad,0,0,0,0,yawRateRadS,simTimeMs);
    }
    /** Intrinsic ZYX orientation and angular rates in the configured robot frame, radians. */
    public synchronized void update(double yawRad,double pitchRad,double rollRad,double omegaX,double omegaY,double omegaZ,long simTimeMs) {
        for(double value:new double[]{yawRad,pitchRad,rollRad,omegaX,omegaY,omegaZ})if(!Double.isFinite(value))throw new IllegalArgumentException("IMU pose/rate must be finite");
        if (simTimeMs < this.simTimeMs) throw new IllegalArgumentException("IMU simulation time moved backwards");
        this.simTimeMs = simTimeMs;
        samples.push(simTimeMs, new Sample(yawRad,pitchRad,rollRad,omegaX,omegaY,omegaZ,simTimeMs,
            gaussian(params.angleNoiseRad),gaussian(params.angleNoiseRad),gaussian(params.angleNoiseRad),
            gaussian(params.gyroNoiseRadS),gaussian(params.gyroNoiseRadS),gaussian(params.gyroNoiseRadS)));
    }

    private Sample observed() {
        Sample delayed = samples.read(simTimeMs, latencyMs);
        return delayed != null ? delayed : new Sample(0,0,0,0,0,0,0,0,0,0,0,0,0);
    }

    @Override public synchronized boolean initialize(Parameters parameters) {
        this.parameters = parameters;
        return true;
    }

    // The simulated bus cost is paid outside the lock so the physics thread can keep publishing samples.
    @Override public YawPitchRollAngles getRobotYawPitchRollAngles() {
        BusCost.block(readCostMs());
        return readAngles();
    }

    @Override public AngularVelocity getRobotAngularVelocity(AngleUnit unit) {
        BusCost.block(readCostMs());
        return readRates(unit);
    }

    private synchronized double readCostMs() { return params.readCostMs; }

    private synchronized YawPitchRollAngles readAngles() {
        Sample sample = observed();
        double elapsedS = Math.max(0, sample.timeMs - yawZeroTimeMs) / 1000.0;
        double raw = (sample.yawRad - yawZeroRad) * (1 + params.yawScaleError) + params.yawDriftRadS * elapsedS + sample.noiseYaw;
        double yaw = Math.atan2(Math.sin(raw), Math.cos(raw));
        return new YawPitchRollAngles(Math.toDegrees(yaw),Math.toDegrees(sample.pitchRad + sample.noisePitch),
            Math.toDegrees(sample.rollRad + sample.noiseRoll),sample.timeMs * 1_000_000L);
    }

    private synchronized AngularVelocity readRates(AngleUnit unit) {
        Sample s=observed();double scale=unit==AngleUnit.DEGREES?180/Math.PI:1;
        double bias = params.gyroBiasRadS;
        return new AngularVelocity(unit,(s.omegaX+bias+s.noiseX)*scale,(s.omegaY+bias+s.noiseY)*scale,
            (s.omegaZ*(1 + params.yawScaleError)+bias+s.noiseZ)*scale);
    }

    @Override public synchronized void resetYaw() {
        Sample s = observed();
        yawZeroRad = s.yawRad;
        yawZeroTimeMs = s.timeMs;
    }

    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated IMU \"" + name + "\""; }
    @Override public void close() { }
}
