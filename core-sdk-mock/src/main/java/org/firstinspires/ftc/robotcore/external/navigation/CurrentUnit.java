package org.firstinspires.ftc.robotcore.external.navigation;

public enum CurrentUnit {
    AMPS, MILLIAMPS;

    public double fromAmps(double amps) {
        return this == AMPS ? amps : amps * 1000.0;
    }
}
