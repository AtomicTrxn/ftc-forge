package simcore;

import com.qualcomm.robotcore.hardware.TouchSensor;

/** Boolean state, driven by simulated collision in later phases (per R1). */
public class SimTouchSensor implements TouchSensor {
    private final String name;
    private volatile boolean pressed = false;
    private volatile double readCostMs;
    /** Time each read blocks the caller, modelling the hub transaction. */
    public void setReadCostMs(double ms) { if (!(ms >= 0 && ms <= 50)) throw new IllegalArgumentException("Sensor read cost must be 0..50 ms"); readCostMs = ms; }
    public SimTouchSensor(String name) { this.name = name; }

    public void setPressed(boolean pressed) { this.pressed = pressed; }
    @Override public boolean isPressed() { BusCost.block(readCostMs); return pressed; }
    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated touch sensor \"" + name + "\""; }
    @Override public void close() { }
}
