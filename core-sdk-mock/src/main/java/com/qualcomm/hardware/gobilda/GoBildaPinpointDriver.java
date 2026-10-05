package com.qualcomm.hardware.gobilda;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import simcore.OdometryTracker;
import simcore.PoseSink;

/**
 * Simulated goBILDA Pinpoint. Public surface follows the vendor driver
 * (goBILDA-Official/goBILDA-Pinpoint-Odometry-Computer-FTC-Driver, GoBildaPinpointDriver.java).
 * Reports the chassis pose from the physics world; with no physics (legacy motor-only executor) it stays at the
 * start pose. Pod offsets, encoder settings, I2C latency and pod faults are accepted but not modeled.
 */
public class GoBildaPinpointDriver implements HardwareDevice, PoseSink {
    public enum DeviceStatus { NOT_READY, READY, CALIBRATING, FAULT_X_POD_NOT_DETECTED,
        FAULT_Y_POD_NOT_DETECTED, FAULT_NO_PODS_DETECTED, FAULT_IMU_RUNAWAY, FAULT_BAD_READ }
    public enum EncoderDirection { FORWARD, REVERSED }
    public enum GoBildaOdometryPods { goBILDA_SWINGARM_POD, goBILDA_4_BAR_POD }
    public enum ReadData { ONLY_UPDATE_HEADING }

    private final String name;
    private final OdometryTracker tracker = new OdometryTracker();
    private OdometryTracker.Pose pose = new OdometryTracker.Pose(0, 0, 0);
    private OdometryTracker.Pose velocity = new OdometryTracker.Pose(0, 0, 0);
    private double xOffsetMm, yOffsetMm;
    private double yawScalar = 1.0;
    private long lastUpdateNanos;
    private double loopTimeUs;

    public GoBildaPinpointDriver(String name) { this.name = name; }

    @Override public void onChassisPose(double x, double y, double yaw, double vx, double vy, double w) {
        tracker.sample(x, y, yaw, vx, vy, w);
    }

    /** Latches the newest simulated pose, like reading the device registers. */
    public void update() {
        pose = tracker.pose();
        velocity = tracker.velocity();
        long now = System.nanoTime();
        if (lastUpdateNanos != 0) loopTimeUs = (now - lastUpdateNanos) / 1000.0;
        lastUpdateNanos = now;
    }

    public void update(ReadData data) { update(); }

    public void setOffsets(double xOffset, double yOffset) { setOffsets(xOffset, yOffset, DistanceUnit.MM); }
    public void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {
        xOffsetMm = unit.toMm(xOffset);
        yOffsetMm = unit.toMm(yOffset);
    }
    public float getXOffset(DistanceUnit unit) { return (float) unit.fromMm(xOffsetMm); }
    public float getYOffset(DistanceUnit unit) { return (float) unit.fromMm(yOffsetMm); }

    public void recalibrateIMU() { }
    public void resetPosAndIMU() { tracker.reset(); update(); }
    public void setEncoderDirections(EncoderDirection x, EncoderDirection y) { }
    public void setEncoderResolution(GoBildaOdometryPods pods) { }
    public void setEncoderResolution(double ticksPerMm) { }
    public void setEncoderResolution(double ticksPerUnit, DistanceUnit unit) { }
    public void setYawScalar(double scalar) { yawScalar = scalar; }
    public float getYawScalar() { return (float) yawScalar; }

    public Pose2D setPosition(Pose2D pos) {
        tracker.set(new OdometryTracker.Pose(pos.getX(DistanceUnit.METER), pos.getY(DistanceUnit.METER),
            pos.getHeading(AngleUnit.RADIANS)));
        update();
        return pos;
    }
    public void setPosX(double posX, DistanceUnit unit) {
        setPosition(new Pose2D(DistanceUnit.MM, unit.toMm(posX), getPosY(DistanceUnit.MM), AngleUnit.RADIANS, getHeading()));
    }
    public void setPosY(double posY, DistanceUnit unit) {
        setPosition(new Pose2D(DistanceUnit.MM, getPosX(DistanceUnit.MM), unit.toMm(posY), AngleUnit.RADIANS, getHeading()));
    }
    public void setHeading(double heading, AngleUnit unit) {
        setPosition(new Pose2D(DistanceUnit.MM, getPosX(DistanceUnit.MM), getPosY(DistanceUnit.MM), unit, heading));
    }

    public int getDeviceID() { return 1; }
    public int getDeviceVersion() { return 1; }
    public DeviceStatus getDeviceStatus() { return DeviceStatus.READY; }
    public int getLoopTime() { return (int) loopTimeUs; }
    public double getFrequency() { return loopTimeUs > 0 ? 1_000_000.0 / loopTimeUs : 0; }
    public int getEncoderX() { return 0; }
    public int getEncoderY() { return 0; }

    /** Millimeters, as in the vendor driver (its no-unit overloads are deprecated there). */
    public double getPosX() { return getPosX(DistanceUnit.MM); }
    public double getPosX(DistanceUnit unit) { return unit.fromMeters(pose.x()); }
    public double getPosY() { return getPosY(DistanceUnit.MM); }
    public double getPosY(DistanceUnit unit) { return unit.fromMeters(pose.y()); }
    /** Unnormalized heading in radians, as in the vendor driver. */
    public double getHeading() { return pose.heading() * yawScalar; }
    public double getHeading(AngleUnit unit) {
        double h = ((getHeading() + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
        return unit.fromRadians(h);
    }
    public double getHeading(UnnormalizedAngleUnit unit) { return unit.fromRadians(getHeading()); }
    public double getVelX() { return getVelX(DistanceUnit.MM); }
    public double getVelX(DistanceUnit unit) { return unit.fromMeters(velocity.x()); }
    public double getVelY() { return getVelY(DistanceUnit.MM); }
    public double getVelY(DistanceUnit unit) { return unit.fromMeters(velocity.y()); }
    public double getHeadingVelocity() { return velocity.heading(); }
    public double getHeadingVelocity(UnnormalizedAngleUnit unit) { return unit.fromRadians(velocity.heading()); }

    public Pose2D getPosition() {
        return new Pose2D(DistanceUnit.MM, getPosX(DistanceUnit.MM), getPosY(DistanceUnit.MM),
            AngleUnit.RADIANS, getHeading(AngleUnit.RADIANS));
    }
    public Pose2D getVelocity() {
        return new Pose2D(DistanceUnit.MM, getVelX(DistanceUnit.MM), getVelY(DistanceUnit.MM),
            AngleUnit.RADIANS, getHeadingVelocity());
    }

    @Override public String getDeviceName() { return "goBILDA Pinpoint Odometry Computer"; }
    @Override public String getConnectionInfo() { return "Simulated Pinpoint \"" + name + "\""; }
    @Override public void close() { }
}
