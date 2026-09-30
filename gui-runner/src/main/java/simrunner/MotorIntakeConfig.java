package simrunner;

import com.jme3.math.Vector3f;
import com.qualcomm.robotcore.hardware.HardwareMap;
import simcore.SimDcMotorEx;
import java.util.List;
import java.util.Map;

/** Capture requires actual shaft rotation in the configured inward direction. */
public record MotorIntakeConfig(String motor, double shaftSign, double minSpeedRadS,
                                Vector3f point, float captureRadiusM) {
    public MotorIntakeConfig {
        if (motor == null || motor.isBlank() || Math.abs(shaftSign) != 1 || !Double.isFinite(minSpeedRadS) || minSpeedRadS <= 0
            || !Float.isFinite(captureRadiusM) || captureRadiusM <= 0 || point == null
            || !Float.isFinite(point.x) || !Float.isFinite(point.y) || !Float.isFinite(point.z))
            throw new IllegalArgumentException("Invalid motor intake configuration");
        point = point.clone();
    }
    @Override public Vector3f point() { return point.clone(); }
    static MotorIntakeConfig parse(Map<String, Object> values) {
        if (!(values.get("point_xyz_m") instanceof List<?> xyz) || xyz.size() != 3
            || xyz.stream().anyMatch(v -> !(v instanceof Number)))
            throw new IllegalArgumentException("intake.point_xyz_m requires three numeric URDF coordinates");
        double[] p = xyz.stream().mapToDouble(v -> ((Number) v).doubleValue()).toArray();
        return new MotorIntakeConfig((String) values.get("motor"), DifferentialDriveConfig.number(values, "shaft_sign"),
            DifferentialDriveConfig.number(values, "min_speed_rad_s"), ImportedRobotScene.position(p),
            (float) DifferentialDriveConfig.number(values, "capture_radius_m"));
    }
    boolean active(HardwareMap map) { return map.get(SimDcMotorEx.class, motor).getOmegaRadS() * shaftSign >= minSpeedRadS; }
}
