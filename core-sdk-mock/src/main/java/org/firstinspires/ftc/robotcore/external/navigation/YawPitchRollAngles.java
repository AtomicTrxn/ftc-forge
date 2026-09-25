package org.firstinspires.ftc.robotcore.external.navigation;

public class YawPitchRollAngles {
    private final double yawDeg, pitchDeg, rollDeg;
    private final long acquisitionNanoTime;

    public YawPitchRollAngles(double yawDeg, double pitchDeg, double rollDeg, long acquisitionNanoTime) {
        this.yawDeg = yawDeg;
        this.pitchDeg = pitchDeg;
        this.rollDeg = rollDeg;
        this.acquisitionNanoTime = acquisitionNanoTime;
    }

    public double getYaw(AngleUnit unit) { return unit.fromDegrees(yawDeg); }
    public double getPitch(AngleUnit unit) { return unit.fromDegrees(pitchDeg); }
    public double getRoll(AngleUnit unit) { return unit.fromDegrees(rollDeg); }
    public long getAcquisitionTime() { return acquisitionNanoTime; }
}
