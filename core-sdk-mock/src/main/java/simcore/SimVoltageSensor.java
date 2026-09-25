package simcore;

import com.qualcomm.robotcore.hardware.VoltageSensor;

/** Returns a constant nominal voltage -- real battery sag (R4) is Phase 3's job. */
public class SimVoltageSensor implements VoltageSensor {
    private final String name;
    public SimVoltageSensor(String name) { this.name = name; }

    @Override public double getVoltage() { return 12.6; }
    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated voltage sensor \"" + name + "\""; }
    @Override public void close() { }
}
