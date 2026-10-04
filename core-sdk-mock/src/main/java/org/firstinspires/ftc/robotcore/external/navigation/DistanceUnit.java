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

    public double toMm(double value) {
        switch (this) {
            case MM: return value;
            case CM: return value * 10.0;
            case METER: return value * 1000.0;
            case INCH: return value * 25.4;
            default: throw new IllegalStateException();
        }
    }

    public double fromMeters(double meters) { return fromMm(meters * 1000.0); }
    public double toMeters(double value) { return toMm(value) / 1000.0; }
}
