package org.firstinspires.ftc.vision.apriltag;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/** Cached geometric observations from configured scene cameras; no image processing. */
public class AprilTagProcessor {

    public static class Builder {
        private DistanceUnit distance=DistanceUnit.INCH;
        private AngleUnit angle=AngleUnit.DEGREES;
        public Builder setDrawAxes(boolean v) { return this; }
        public Builder setDrawCubeProjection(boolean v) { return this; }
        public Builder setDrawTagOutline(boolean v) { return this; }
        public Builder setOutputUnits(DistanceUnit distance,AngleUnit angle){this.distance=java.util.Objects.requireNonNull(distance);this.angle=java.util.Objects.requireNonNull(angle);return this;}
        public AprilTagProcessor build() { return new AprilTagProcessor(distance,angle); }
    }
    private final DistanceUnit distance;
    private final AngleUnit angle;
    private volatile java.util.function.Supplier<List<AprilTagDetection>> source=List::of;
    private AprilTagProcessor(DistanceUnit distance,AngleUnit angle){this.distance=distance;this.angle=angle;}
    public void attachSimulatedCamera(WebcamName camera,java.util.function.BooleanSupplier active){source=()->active.getAsBoolean()?camera.simulatedDetections():List.of();}

    public List<AprilTagDetection> getDetections() {
        return source.get().stream().map(original->{var d=new AprilTagDetection();d.id=original.id;d.metadata=original.metadata;d.frameAcquisitionNanoTime=original.frameAcquisitionNanoTime;
            var p=original.ftcPose;if(p!=null)d.ftcPose=new AprilTagPoseFtc(distance.fromMeters(p.x),distance.fromMeters(p.y),distance.fromMeters(p.z),angle.fromDegrees(Math.toDegrees(p.yaw)),angle.fromDegrees(Math.toDegrees(p.pitch)),angle.fromDegrees(Math.toDegrees(p.roll)),distance.fromMeters(p.range),angle.fromDegrees(Math.toDegrees(p.bearing)),angle.fromDegrees(Math.toDegrees(p.elevation)));return d;}).toList();
    }
}
