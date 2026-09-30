package simrunner;

import java.util.List;
import java.util.Map;
import com.qualcomm.robotcore.hardware.HardwareMap;
import simcore.SimDcMotorEx;
import physics.DifferentialKinematics;
import physics.MecanumKinematics;

/** Physical mounting signs remain independent of an OpMode's Direction setting. */
public record DifferentialDriveConfig(String leftMotor, String rightMotor, double trackWidthM,
                                      double wheelRadiusM, double leftShaftSign, double rightShaftSign) {
    public DifferentialDriveConfig {
        if (leftMotor == null || leftMotor.isBlank() || rightMotor == null || rightMotor.isBlank() || leftMotor.equals(rightMotor))
            throw new IllegalArgumentException("Differential drive needs distinct left_motor and right_motor names");
        if (!Double.isFinite(trackWidthM) || trackWidthM <= 0 || !Double.isFinite(wheelRadiusM) || wheelRadiusM <= 0)
            throw new IllegalArgumentException("Drive track_width_m and wheel_radius_m must be finite and positive");
        if (Math.abs(leftShaftSign) != 1 || Math.abs(rightShaftSign) != 1)
            throw new IllegalArgumentException("Drive shaft signs must be +1 or -1");
    }
    static DifferentialDriveConfig parse(Map<String, Object> values) {
        if (!"differential".equals(values.get("type"))) throw new IllegalArgumentException("drive.type must be differential; omit drive for default Mecanum");
        return new DifferentialDriveConfig((String) values.get("left_motor"), (String) values.get("right_motor"),
            number(values, "track_width_m"), number(values, "wheel_radius_m"),
            number(values, "left_shaft_sign"), number(values, "right_shaft_sign"));
    }
    static double number(Map<String, Object> values, String key) {
        if (!(values.get(key) instanceof Number n)) throw new IllegalArgumentException("Missing numeric " + key);
        return n.doubleValue();
    }
    List<String> motorNames() { return List.of(leftMotor, rightMotor); }
    MecanumKinematics.ChassisVelocity velocity(HardwareMap map) {
        return new DifferentialKinematics(trackWidthM).forwardFromWheelSpeeds(
            map.get(SimDcMotorEx.class, leftMotor).getOmegaRadS() * leftShaftSign * wheelRadiusM,
            map.get(SimDcMotorEx.class, rightMotor).getOmegaRadS() * rightShaftSign * wheelRadiusM);
    }
}
