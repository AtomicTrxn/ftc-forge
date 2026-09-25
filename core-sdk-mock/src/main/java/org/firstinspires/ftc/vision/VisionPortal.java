package org.firstinspires.ftc.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/** No-op stub (R1 amendment) -- reports the camera as unavailable, never produces frames. */
public class VisionPortal {

    public enum CameraState { CAMERA_DEVICE_CLOSED, ERROR }

    public static class Builder {
        public Builder setCamera(WebcamName webcamName) { return this; }
        public Builder addProcessor(AprilTagProcessor processor) { return this; }
        public VisionPortal build() { return new VisionPortal(); }
    }

    private VisionPortal() { }

    public CameraState getCameraState() { return CameraState.CAMERA_DEVICE_CLOSED; }
    public void close() { }
}
