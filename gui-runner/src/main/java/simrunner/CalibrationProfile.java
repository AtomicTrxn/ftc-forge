package simrunner;

import com.qualcomm.robotcore.hardware.HardwareMap;
import physics.BatteryModel;
import simcore.HardwareMapBuilder;
import simcore.MiniJson;
import simcore.SimDcMotorEx;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;

/** Validated recording-derived parameters, applied only when sim.config opts in. */
final class CalibrationProfile {
    record Motor(double staticTorqueNm, double viscousBNmS) { }
    record Drive(double responseTimeS, double maxAccelMps2,
                 double yawResponseTimeS, double maxYawAccelRadps2) { }
    final double vInternal, rBattery;
    final Map<String, Motor> motors;
    final Drive drive;

    private CalibrationProfile(double vInternal, double rBattery, Map<String, Motor> motors, Drive drive) {
        this.vInternal = vInternal;
        this.rBattery = rBattery;
        this.motors = motors;
        this.drive = drive;
    }

    @SuppressWarnings("unchecked")
    static CalibrationProfile load(Path path) throws IOException {
        Map<String, Object> root = MiniJson.parseObject(Files.readString(path));
        Map<String, Object> battery = (Map<String, Object>) root.get("battery");
        if (battery == null) throw new IllegalArgumentException("Calibration profile needs battery parameters");
        double v = number(battery, "v_internal", false), r = number(battery, "r_battery", true);
        if (v > 18 || r > 2) throw new IllegalArgumentException("Calibration battery parameters out of bounds");
        Map<String, Motor> motors = new java.util.LinkedHashMap<>();
        Map<String, Object> entries = (Map<String, Object>) root.get("motors");
        if (entries != null) for (var entry : entries.entrySet()) {
            Map<String, Object> values = (Map<String, Object>) entry.getValue();
            motors.put(entry.getKey(), new Motor(number(values, "tau_static_nm", true),
                number(values, "viscous_b_nm_s_per_rad", true)));
        }
        Drive drive = null;
        Map<String, Object> values = (Map<String, Object>) root.get("drive");
        if (values != null) drive = new Drive(number(values, "response_time_s", false),
            number(values, "max_accel_mps2", false), number(values, "yaw_response_time_s", false),
            number(values, "max_yaw_accel_radps2", false));
        return new CalibrationProfile(v, r, Map.copyOf(motors), drive);
    }

    void applyHardware(HardwareMap map) {
        for (var entry : motors.entrySet()) {
            SimDcMotorEx motor = map.tryGet(SimDcMotorEx.class, entry.getKey());
            if (motor == null) throw new IllegalArgumentException("Calibrated motor missing from HardwareMap: " + entry.getKey());
        }
        BatteryModel battery = HardwareMapBuilder.getBatteryModel();
        battery.vInternal = vInternal;
        battery.rBattery = rBattery;
        for (var entry : motors.entrySet()) {
            Motor spec = entry.getValue();
            map.get(SimDcMotorEx.class, entry.getKey()).configureFriction(spec.staticTorqueNm(), spec.viscousBNmS());
        }
    }

    void applyDrive(PhysicsWorld world) {
        if (drive != null) world.configureDrive(drive.responseTimeS(), drive.maxAccelMps2(),
            drive.yawResponseTimeS(), drive.maxYawAccelRadps2());
    }

    private static double number(Map<String, Object> values, String key, boolean allowZero) {
        Object raw = values.get(key);
        if (!(raw instanceof Number number)) throw new IllegalArgumentException("Missing calibration number " + key);
        double value = number.doubleValue();
        if (!Double.isFinite(value) || (allowZero ? value < 0 : value <= 0))
            throw new IllegalArgumentException("Invalid calibration number " + key);
        return value;
    }
}
