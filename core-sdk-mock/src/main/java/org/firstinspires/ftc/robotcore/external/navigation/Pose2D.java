package org.firstinspires.ftc.robotcore.external.navigation;

/** Immutable 2D pose with explicit units, matching the SDK's Pose2D accessors. */
public class Pose2D {
    private final DistanceUnit distanceUnit;
    private final AngleUnit angleUnit;
    private final double x, y, heading;

    public Pose2D(DistanceUnit distanceUnit, double x, double y, AngleUnit angleUnit, double heading) {
        this.distanceUnit = distanceUnit;
        this.angleUnit = angleUnit;
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX(DistanceUnit unit) { return unit.fromMm(distanceUnit.toMm(x)); }
    public double getY(DistanceUnit unit) { return unit.fromMm(distanceUnit.toMm(y)); }
    public double getHeading(AngleUnit unit) { return unit.fromRadians(angleUnit.toRadians(heading)); }
}
