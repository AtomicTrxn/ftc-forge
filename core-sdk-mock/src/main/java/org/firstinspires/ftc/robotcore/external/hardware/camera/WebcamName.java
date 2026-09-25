package org.firstinspires.ftc.robotcore.external.hardware.camera;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/** No-op stub -- vision is deferred (R1), but the class must exist for team code to compile. */
public class WebcamName implements HardwareDevice {
    private final String name;
    public WebcamName(String name) { this.name = name; }
    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated webcam (no camera output)"; }
    @Override public void close() { }
}
