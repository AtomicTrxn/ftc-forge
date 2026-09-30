package simrunner;

import simcore.MiniJson;
import physics.ServoModel;
import java.util.LinkedHashMap;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Parses a team project's sim.config marker file (per R3's design): the minimal, low-friction
 * "point the simulator at your folder" mechanism -- no entry-OpMode field needed, since
 * discovery is annotation-driven (see OpModeDiscovery).
 */
public class SimConfig {
    public String sourceRoot = "TeamCode/src/main/java";
    public String robotConfig;
    public String presetMotors;
    public String urdf;
    public String calibration;
    public DifferentialDriveConfig drive;
    public MotorIntakeConfig intake;
    public TireDriveConfig tires;
    public FlexibleIntakeConfig flexibleIntake;
    public double startHeightM = .1;
    public Double totalMassKg;
    public int vhacdMaxHulls = 8;
    public long imuLatencyMs = 8;
    public final List<String> extraClasspath = new ArrayList<>();
    public final Map<String, ServoModel.Spec> servoPhysics = new LinkedHashMap<>();

    @SuppressWarnings("unchecked")
    public static SimConfig load(Path projectRoot) throws IOException {
        Path configPath = projectRoot.resolve("sim.config");
        Map<String, Object> root = MiniJson.parseObject(Files.readString(configPath));
        SimConfig config = new SimConfig();
        if (root.containsKey("sourceRoot")) config.sourceRoot = (String) root.get("sourceRoot");
        if (root.containsKey("robotConfig")) config.robotConfig = (String) root.get("robotConfig");
        if (root.containsKey("presetMotors")) config.presetMotors = (String) root.get("presetMotors");
        if (root.containsKey("urdf")) config.urdf = (String) root.get("urdf");
        if (root.containsKey("calibration")) config.calibration = (String) root.get("calibration");
        if (root.containsKey("total_mass_kg")) config.totalMassKg = ((Number) root.get("total_mass_kg")).doubleValue();
        if (root.containsKey("vhacd_max_hulls")) config.vhacdMaxHulls = ((Number) root.get("vhacd_max_hulls")).intValue();
        if (root.containsKey("imu_latency_ms")) config.imuLatencyMs = ((Number) root.get("imu_latency_ms")).longValue();
        if (config.vhacdMaxHulls < 1 || config.vhacdMaxHulls > 16)
            throw new IllegalArgumentException("vhacd_max_hulls must be between 1 and 16");
        if (config.imuLatencyMs < 0 || config.imuLatencyMs > 200)
            throw new IllegalArgumentException("imu_latency_ms must be between 0 and 200");
        if (root.containsKey("extraClasspath")) {
            for (Object o : (List<Object>) root.get("extraClasspath")) {
                config.extraClasspath.add((String) o);
            }
        }
        if (root.containsKey("servoPhysics")) {
            Map<String, Object> entries = (Map<String, Object>) root.get("servoPhysics");
            for (var entry : entries.entrySet()) {
                Map<String, Object> values = (Map<String, Object>) entry.getValue();
                config.servoPhysics.put(entry.getKey(), new ServoModel.Spec(
                    requiredNumber(values, "stall_torque_nm"), requiredNumber(values, "no_load_speed_rad_s"),
                    requiredNumber(values, "travel_rad"), requiredNumber(values, "position_gain_per_s"),
                    requiredNumber(values, "velocity_gain_nm_per_rad_s"), requiredNumber(values, "deadband_rad")));
            }
        }
        if (root.containsKey("drive")) config.drive = DifferentialDriveConfig.parse((Map<String, Object>) root.get("drive"));
        if (root.containsKey("tires")) {
            config.tires = TireDriveConfig.parse((Map<String, Object>) root.get("tires"));
            if (config.drive == null || config.urdf == null) throw new IllegalArgumentException("tires requires differential drive and URDF");
        }
        if (root.containsKey("intake")) config.intake = MotorIntakeConfig.parse((Map<String, Object>) root.get("intake"));
        if (root.containsKey("flexible_intake")) {
            config.flexibleIntake = FlexibleIntakeConfig.parse((Map<String,Object>)root.get("flexible_intake"));
            if (config.urdf == null || config.intake == null) throw new IllegalArgumentException("flexible_intake requires URDF and motor intake");
        }
        if (root.containsKey("start_height_m")) config.startHeightM = ((Number) root.get("start_height_m")).doubleValue();
        if (!Double.isFinite(config.startHeightM) || config.startHeightM < 0)
            throw new IllegalArgumentException("start_height_m must be finite and nonnegative");
        return config;
    }
    private static double requiredNumber(Map<String, Object> values, String key) {
        Object value = values.get(key);
        if (!(value instanceof Number number))
            throw new IllegalArgumentException("servoPhysics requires numeric " + key);
        return number.doubleValue();
    }
}
