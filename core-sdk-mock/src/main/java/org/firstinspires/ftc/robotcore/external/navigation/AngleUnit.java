package org.firstinspires.ftc.robotcore.external.navigation;

public enum AngleUnit {
    DEGREES, RADIANS;

    public double fromDegrees(double degrees) {
        return this == DEGREES ? degrees : Math.toRadians(degrees);
    }
}
