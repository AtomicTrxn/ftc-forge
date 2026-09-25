package simcore;

import com.qualcomm.robotcore.hardware.VoltageSensor;

/** Reflects the real, shared battery-sag voltage HardwareMapBuilder computes each tick (R4), not a constant. */
public class SimVoltageSensor implements VoltageSensor {
    private final String name;
    private volatile double voltage = 12.6;

    public SimVoltageSensor(String name) { this.name = name; }

    public void setVoltage(double v) { this.voltage = v; }

    @Override public double getVoltage() { return voltage; }
    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated voltage sensor \"" + name + "\""; }
    @Override public void close() { }
}
