package simcore;

import com.qualcomm.robotcore.hardware.ColorSensor;

/** Trivial stub -- realistic behavior needs Phase 4's field/game-piece geometry (per R1). */
public class SimColorSensor implements ColorSensor {
    private final String name;
    public SimColorSensor(String name) { this.name = name; }

    @Override public int red() { return 0; }
    @Override public int green() { return 0; }
    @Override public int blue() { return 0; }
    @Override public int alpha() { return 0; }
    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated color sensor \"" + name + "\""; }
    @Override public void close() { }
}
