package org.firstinspires.ftc.robotcore.external.navigation;

public class AngularVelocity {
    public final double xRotationRate, yRotationRate, zRotationRate;
    private final AngleUnit unit;

    public AngularVelocity(AngleUnit unit, double x, double y, double z) {
        this.unit = unit;
        this.xRotationRate = x;
        this.yRotationRate = y;
        this.zRotationRate = z;
    }

    public AngleUnit getUnit() { return unit; }
}
