package simcore;

import com.qualcomm.robotcore.hardware.TouchSensor;

/** Boolean state, driven by simulated collision in later phases (per R1). */
public class SimTouchSensor implements TouchSensor {
    private final String name;
    private boolean pressed = false;
    public SimTouchSensor(String name) { this.name = name; }

    public void setPressed(boolean pressed) { this.pressed = pressed; }
    @Override public boolean isPressed() { return pressed; }
    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated touch sensor \"" + name + "\""; }
    @Override public void close() { }
}
