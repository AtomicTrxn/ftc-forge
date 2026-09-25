package org.firstinspires.ftc.robotcore.external.navigation;

public enum DistanceUnit {
    MM, CM, METER, INCH;

    public double fromMm(double mm) {
        switch (this) {
            case MM: return mm;
            case CM: return mm / 10.0;
            case METER: return mm / 1000.0;
            case INCH: return mm / 25.4;
            default: throw new IllegalStateException();
        }
    }
}
