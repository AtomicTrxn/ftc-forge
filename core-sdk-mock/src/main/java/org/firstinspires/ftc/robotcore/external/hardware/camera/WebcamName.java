package org.firstinspires.ftc.robotcore.external.hardware.camera;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/** Hardware binding for configured geometric observations; no camera image frames. */
public class WebcamName implements HardwareDevice {
    private final String name;
    private volatile boolean configured;
    private volatile java.util.List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> detections=java.util.List.of();
    public WebcamName(String name) { this.name = name; }
    public void setSimulatedDetections(java.util.List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> values){detections=java.util.List.copyOf(values);configured=true;}
    public java.util.List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> simulatedDetections(){return detections;}
    public boolean hasSimulatedCamera(){return configured;}
    public void clearSimulatedCamera(){detections=java.util.List.of();configured=false;}
    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return configured?"Simulated geometric AprilTag camera (no image frames)":"Unconfigured webcam (no camera output)"; }
    @Override public void close() { }
}
