package org.firstinspires.ftc.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/** Controls configured geometric camera observations; no rendered camera frames. */
public class VisionPortal {

    public enum CameraState { CAMERA_DEVICE_CLOSED, CAMERA_DEVICE_READY, STREAMING, ERROR }

    public static class Builder {
        private WebcamName camera;
        private final java.util.List<AprilTagProcessor> processors=new java.util.ArrayList<>();
        public Builder setCamera(WebcamName webcamName) {camera=java.util.Objects.requireNonNull(webcamName); return this; }
        public Builder addProcessor(AprilTagProcessor processor) {processors.add(java.util.Objects.requireNonNull(processor)); return this; }
        public VisionPortal build() { return new VisionPortal(camera,processors); }
    }
    private final WebcamName camera;
    private final java.util.Map<AprilTagProcessor,Boolean> enabled=new java.util.concurrent.ConcurrentHashMap<>();
    private volatile boolean closed,streaming=true;
    private VisionPortal(WebcamName camera,java.util.List<AprilTagProcessor> processors){this.camera=camera;for(var processor:processors){enabled.put(processor,true);if(camera!=null)processor.attachSimulatedCamera(camera,()->!closed&&streaming&&Boolean.TRUE.equals(enabled.get(processor)));}}
    public CameraState getCameraState() {return closed||camera==null||!camera.hasSimulatedCamera()?CameraState.CAMERA_DEVICE_CLOSED:streaming?CameraState.STREAMING:CameraState.CAMERA_DEVICE_READY;}
    public void setProcessorEnabled(AprilTagProcessor processor,boolean value){if(!enabled.containsKey(processor))throw new IllegalArgumentException("Processor does not belong to this portal");enabled.put(processor,value);}
    public void stopStreaming(){streaming=false;}
    public void resumeStreaming(){if(!closed)streaming=true;}
    public void close() {closed=true;}
}
