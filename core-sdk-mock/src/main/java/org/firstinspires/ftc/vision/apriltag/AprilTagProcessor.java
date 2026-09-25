package org.firstinspires.ftc.vision.apriltag;

import java.util.Collections;
import java.util.List;

/**
 * No-op stub. Per R1's amendment, "vision deferred" means no simulated camera output,
 * not that the classes don't exist -- real OpModes that construct this in init() must
 * still compile and run; it just always reports zero detections.
 */
public class AprilTagProcessor {

    public static class Builder {
        public Builder setDrawAxes(boolean v) { return this; }
        public Builder setDrawCubeProjection(boolean v) { return this; }
        public Builder setDrawTagOutline(boolean v) { return this; }
        public AprilTagProcessor build() { return new AprilTagProcessor(); }
    }

    private AprilTagProcessor() { }

    public List<AprilTagDetection> getDetections() {
        return Collections.emptyList();
    }
}
