package org.firstinspires.ftc.robotcore.external.navigation;

/** Angle unit whose values are not wrapped into a single revolution (used for headings and rates). */
public enum UnnormalizedAngleUnit {
    DEGREES, RADIANS;

    public double fromRadians(double radians) {
        return this == RADIANS ? radians : Math.toDegrees(radians);
    }

    public double toRadians(double value) {
        return this == RADIANS ? value : Math.toRadians(value);
    }
}
