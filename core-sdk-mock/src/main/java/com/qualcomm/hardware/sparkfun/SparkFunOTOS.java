package com.qualcomm.hardware.sparkfun;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import simcore.OdometryTracker;
import simcore.PoseSink;

/**
 * Simulated SparkFun Qwiic OTOS. Public surface follows SparkFun's FTC library (SparkFunOTOS.java);
 * the chassis pose comes from the physics world and stays at the start pose headless. Scalars,
 * signal-processing flags, self-test and tracking noise/dropouts are accepted but not modeled.
 */
public class SparkFunOTOS implements HardwareDevice, PoseSink {

    public static class Pose2D {
        public double x, y, h;
        public Pose2D() { }
        public Pose2D(double x, double y, double h) { this.x = x; this.y = y; this.h = h; }
        public void set(Pose2D pose) { x = pose.x; y = pose.y; h = pose.h; }
    }

    public static class Version {
        public byte minor, major;
        public Version() { }
        public Version(byte value) { set(value); }
        public void set(byte value) { minor = (byte) (value & 0xF); major = (byte) ((value >> 4) & 0xF); }
        public byte get() { return (byte) ((major << 4) | minor); }
    }

    public static class SignalProcessConfig {
        public boolean enLut, enAcc, enRot, enVar;
        public SignalProcessConfig() { }
        public SignalProcessConfig(byte value) { set(value); }
        public void set(byte value) {
            enLut = (value & 1) != 0; enAcc = (value & 2) != 0; enRot = (value & 4) != 0; enVar = (value & 8) != 0;
        }
        public byte get() { return (byte) ((enLut ? 1 : 0) | (enAcc ? 2 : 0) | (enRot ? 4 : 0) | (enVar ? 8 : 0)); }
    }

    public static class Status {
        public boolean warnTiltAngle, warnOpticalTracking, errorPaa, errorLsm;
        public Status() { }
        public Status(byte value) { set(value); }
        public void set(byte value) {
            warnTiltAngle = (value & 1) != 0; warnOpticalTracking = (value & 2) != 0;
            errorPaa = (value & 4) != 0; errorLsm = (value & 8) != 0;
        }
        public byte get() {
            return (byte) ((warnTiltAngle ? 1 : 0) | (warnOpticalTracking ? 2 : 0) | (errorPaa ? 4 : 0) | (errorLsm ? 8 : 0));
        }
    }

    private final String name;
    private final OdometryTracker tracker = new OdometryTracker();
    private DistanceUnit linearUnit = DistanceUnit.INCH;
    private AngleUnit angularUnit = AngleUnit.DEGREES;
    private double linearScalar = 1.0, angularScalar = 1.0;
    private Pose2D offset = new Pose2D();
    private SignalProcessConfig signalProcessConfig = new SignalProcessConfig((byte) 0x0F);

    public SparkFunOTOS(String name) { this.name = name; }

    @Override public void onChassisPose(double x, double y, double yaw, double vx, double vy, double w) {
        tracker.sample(x, y, yaw, vx, vy, w);
    }

    public boolean begin() { return true; }
    public boolean isConnected() { return true; }
    public void getVersionInfo(Version hw, Version fw) { hw.set((byte) 0x10); fw.set((byte) 0x10); }
    public boolean selfTest() { return true; }
    public boolean calibrateImu() { return true; }
    public boolean calibrateImu(int numSamples, boolean waitUntilDone) { return true; }
    public int getImuCalibrationProgress() { return 0; }

    public DistanceUnit getLinearUnit() { return linearUnit; }
    public void setLinearUnit(DistanceUnit unit) { linearUnit = unit; }
    public AngleUnit getAngularUnit() { return angularUnit; }
    public void setAngularUnit(AngleUnit unit) { angularUnit = unit; }
    public double getLinearScalar() { return linearScalar; }
    public boolean setLinearScalar(double scalar) { linearScalar = scalar; return true; }
    public double getAngularScalar() { return angularScalar; }
    public boolean setAngularScalar(double scalar) { angularScalar = scalar; return true; }

    public void resetTracking() { tracker.reset(); }
    public SignalProcessConfig getSignalProcessConfig() { return signalProcessConfig; }
    public void setSignalProcessConfig(SignalProcessConfig config) { signalProcessConfig = config; }
    public Status getStatus() { return new Status(); }
    public Pose2D getOffset() { return offset; }
    public void setOffset(Pose2D pose) { offset = pose; }

    public Pose2D getPosition() {
        OdometryTracker.Pose p = tracker.pose();
        return out(p.x(), p.y(), p.heading(), angularScalar);
    }
    public void setPosition(Pose2D pose) {
        tracker.set(new OdometryTracker.Pose(linearUnit.toMeters(pose.x), linearUnit.toMeters(pose.y),
            angularUnit.toRadians(pose.h)));
    }
    public Pose2D getVelocity() {
        OdometryTracker.Pose v = tracker.velocity();
        return out(v.x(), v.y(), v.heading(), 1.0);
    }
    public Pose2D getAcceleration() { return new Pose2D(); }
    public Pose2D getPositionStdDev() { return new Pose2D(); }
    public Pose2D getVelocityStdDev() { return new Pose2D(); }
    public Pose2D getAccelerationStdDev() { return new Pose2D(); }

    public void getPosVelAcc(Pose2D pos, Pose2D vel, Pose2D acc) {
        pos.set(getPosition()); vel.set(getVelocity()); acc.set(getAcceleration());
    }
    public void getPosVelAccStdDev(Pose2D pos, Pose2D vel, Pose2D acc) {
        pos.set(getPositionStdDev()); vel.set(getVelocityStdDev()); acc.set(getAccelerationStdDev());
    }
    public void getPosVelAccAndStdDev(Pose2D pos, Pose2D vel, Pose2D acc,
                                      Pose2D posStdDev, Pose2D velStdDev, Pose2D accStdDev) {
        getPosVelAcc(pos, vel, acc);
        getPosVelAccStdDev(posStdDev, velStdDev, accStdDev);
    }

    private Pose2D out(double xMeters, double yMeters, double headingRad, double angularMul) {
        double h = angularUnit.fromRadians(headingRad * angularMul);
        if (angularUnit == AngleUnit.DEGREES) h = ((h + 180) % 360 + 360) % 360 - 180;
        else h = ((h + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
        return new Pose2D(linearUnit.fromMeters(xMeters) * linearScalar, linearUnit.fromMeters(yMeters) * linearScalar, h);
    }

    @Override public String getDeviceName() { return "SparkFun Qwiic Optical Tracking Odometry Sensor"; }
    @Override public String getConnectionInfo() { return "Simulated OTOS \"" + name + "\""; }
    @Override public void close() { }
}
