package simrunner;

import simcore.MiniJson;
import physics.ServoModel;
import java.util.LinkedHashMap;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
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
    ModelProfile robotProfile;
    public Map<String,Object> scenePieces=Map.of();
    List<SceneSensorConfig> sensors=List.of();
    Map<String,Object> fieldBehavior=Map.of();
    public DifferentialDriveConfig drive;
    DriveGeometry driveGeometry;
    public MotorIntakeConfig intake;
    public TireDriveConfig tires;
    RotatingWheelConfig rotatingWheels;
    DriveContactConfig driveContacts;
    public FlexibleIntakeConfig flexibleIntake;
    public TorusRetentionConfig torusRetention;
    public com.jme3.math.Vector3f gamePieceStart;
    public FieldConfig field = FieldConfig.defaults();
    public com.jme3.math.Vector3f robotStart;
    public float robotYawRad;
    public double startHeightM = .1;
    public Double totalMassKg;
    public int vhacdMaxHulls = 8;
    public long imuLatencyMs = 8;
    public long encoderLatencyMs = simcore.SimDcMotorEx.DEFAULT_ENCODER_LATENCY_MS;
    public double batteryInternalVoltageV = 12.6;
    public double batteryInternalResistanceOhm = 0.15;
    public final List<String> extraClasspath = new ArrayList<>();
    public final Map<String, ServoModel.Spec> servoPhysics = new LinkedHashMap<>();
    public final Map<String,String> collisionOmissions = new LinkedHashMap<>();

    /**
     * Applies the electrical and bus settings to a freshly built hardware map. Call before loading a
     * calibration profile: measured battery values from a profile take precedence over these.
     */
    public void applyElectrical(com.qualcomm.robotcore.hardware.HardwareMap map) {
        var battery = simcore.HardwareMapBuilder.getBatteryModel();
        battery.vInternal = batteryInternalVoltageV;
        battery.rBattery = batteryInternalResistanceOhm;
        for (var motor : map.getAll(simcore.SimDcMotorEx.class)) motor.setEncoderLatencyMs(encoderLatencyMs);
    }

    @SuppressWarnings("unchecked")
    public static SimConfig load(Path projectRoot) throws IOException {
        Path configPath = projectRoot.resolve("sim.config");
        Map<String, Object> root = MiniJson.parseObject(Files.readString(configPath));
        return parse(projectRoot,root);
    }

    static SimConfig parse(Path projectRoot, Map<String,Object> input) throws IOException {
        Map<String,Object> root=new LinkedHashMap<>(input);
        try {if(root.get("scene_profile") instanceof String scene)root=SceneProfile.apply(projectRoot.resolve(scene).toRealPath(),root);}catch(Exception e){throw new IOException("Scene profile: "+e.getMessage(),e);}
        ModelProfile model=null;
        if(root.get("robot_model_profile") instanceof String ref)try {
            model=new ModelProfile(projectRoot.resolve(ref),true);
            if(!model.kind.equals("robot"))throw new IllegalArgumentException("Select a robot profile");
            var conflicts=new ArrayList<String>();for(String key:model.runtime.keySet())if(root.containsKey(key))conflicts.add(key);
            if(!conflicts.isEmpty() && !Set.of("profile","project").contains(root.get("model_settings")))throw new IllegalArgumentException("Choose model_settings: profile or project for overlapping settings: "+conflicts);
            if("project".equals(root.get("model_settings"))&&root.containsKey("vhacd_max_hulls")&&!root.get("vhacd_max_hulls").equals(model.runtime.get("vhacd_max_hulls")))throw new IllegalArgumentException("Decomposition changes require review; edit max_hulls in the model profile before using project hull settings");
            boolean projectCalibration="project".equals(root.get("model_settings"))&&root.containsKey("calibration");
            for(var e:model.runtime.entrySet())if(!"project".equals(root.get("model_settings"))||!root.containsKey(e.getKey()))root.put(e.getKey(),e.getValue());
            root.put("urdf",model.artifact("robot").toString());
            if(model.runtime.containsKey("calibration")&&!projectCalibration)root.put("calibration",model.resolve(model.runtime.get("calibration").toString()).toString());
        }catch(Exception e){throw new IOException("Robot model profile: "+e.getMessage(),e);}
        SimConfig config = new SimConfig();
        config.robotProfile=model;
        if(root.containsKey("scene_pieces"))config.scenePieces=FieldPackage.map(root.get("scene_pieces"));
        if(root.containsKey("sensors"))config.sensors=SceneSensorConfig.parse(root.get("sensors"));
        if(root.containsKey("field_behavior")){config.fieldBehavior=FieldPackage.map(root.get("field_behavior"));FieldBehavior.validate(config.fieldBehavior);if(FieldPackage.num(config.fieldBehavior,"schema_version")!=1)throw new IllegalArgumentException("Unsupported field behavior schema");}
        if (root.containsKey("sourceRoot")) config.sourceRoot = (String) root.get("sourceRoot");
        if (root.containsKey("robotConfig")) config.robotConfig = (String) root.get("robotConfig");
        if (root.containsKey("presetMotors")) config.presetMotors = (String) root.get("presetMotors");
        if (root.containsKey("urdf")) config.urdf = (String) root.get("urdf");
        if (root.containsKey("collision_omissions")) {
            var omissions=FieldPackage.map(root.get("collision_omissions"));
            for(var entry:omissions.entrySet()) {
                if(entry.getKey().isBlank() || !(entry.getValue() instanceof String reason) || reason.isBlank())
                    throw new IllegalArgumentException("collision_omissions requires body names and nonempty reasons");
                config.collisionOmissions.put(entry.getKey(),reason);
            }
            if(config.urdf==null)throw new IllegalArgumentException("collision_omissions requires a robot URDF");
        }
        if (root.containsKey("field")) config.field=FieldConfig.parse(FieldPackage.map(root.get("field")));
        if (root.containsKey("robot_start_xyz_m")) config.robotStart=FieldPackage.pos(root.get("robot_start_xyz_m"));
        if (root.containsKey("robot_start_yaw_rad")) config.robotYawRad=(float)FieldPackage.num(root,"robot_start_yaw_rad");
        if(!Float.isFinite(config.robotYawRad))throw new IllegalArgumentException("Robot yaw must be finite");
        float floorTop=0;
        if(config.field.source().equals("imported")&&Path.of(config.field.packagePath()).getFileName().toString().equals("profile.json"))try{var fieldModel=new ModelProfile(projectRoot.resolve(config.field.packagePath()),true);if(!fieldModel.kind.equals("field"))throw new IllegalArgumentException("Select a field profile");floorTop=(float)FieldPackage.num(fieldModel.parameters,"floor_top_m");}catch(Exception e){throw new IOException("Field model profile: "+e.getMessage(),e);}
        if(config.robotStart!=null && config.robotStart.y<floorTop)throw new IllegalArgumentException("Robot start must be above the field surface");
        if (root.containsKey("calibration")) config.calibration = (String) root.get("calibration");
        if (root.containsKey("total_mass_kg")) config.totalMassKg = ((Number) root.get("total_mass_kg")).doubleValue();
        if (config.totalMassKg != null && (!Double.isFinite(config.totalMassKg) || config.totalMassKg <= 0))
            throw new IllegalArgumentException("Total robot mass must be positive and finite.");
        if (root.containsKey("vhacd_max_hulls")) config.vhacdMaxHulls = ((Number) root.get("vhacd_max_hulls")).intValue();
        if (root.containsKey("imu_latency_ms")) config.imuLatencyMs = ((Number) root.get("imu_latency_ms")).longValue();
        if (root.containsKey("encoder_latency_ms")) config.encoderLatencyMs = ((Number) root.get("encoder_latency_ms")).longValue();
        if (root.get("battery") instanceof Map<?, ?> battery) {
            Map<String, Object> values = (Map<String, Object>) battery;
            for (String key : values.keySet())
                if (!key.equals("internal_voltage_v") && !key.equals("internal_resistance_ohm"))
                    throw new IllegalArgumentException("Unknown battery setting: " + key);
            if (values.containsKey("internal_voltage_v")) config.batteryInternalVoltageV = ((Number) values.get("internal_voltage_v")).doubleValue();
            if (values.containsKey("internal_resistance_ohm")) config.batteryInternalResistanceOhm = ((Number) values.get("internal_resistance_ohm")).doubleValue();
        } else if (root.containsKey("battery")) {
            throw new IllegalArgumentException("battery must be an object with internal_voltage_v and/or internal_resistance_ohm");
        }
        if (config.encoderLatencyMs < 0 || config.encoderLatencyMs > 200)
            throw new IllegalArgumentException("encoder_latency_ms must be between 0 and 200");
        if (!(config.batteryInternalVoltageV >= 6 && config.batteryInternalVoltageV <= 18))
            throw new IllegalArgumentException("battery.internal_voltage_v must be between 6 and 18");
        if (!(config.batteryInternalResistanceOhm >= 0 && config.batteryInternalResistanceOhm <= 2))
            throw new IllegalArgumentException("battery.internal_resistance_ohm must be between 0 and 2");
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
        if(root.containsKey("drive_contacts"))config.driveContacts=DriveContactConfig.parse(FieldPackage.map(root.get("drive_contacts")));
        if (root.containsKey("drive_geometry")) {
            if(config.drive!=null)throw new IllegalArgumentException("Differential dimensions belong in drive; drive_geometry is for Mecanum.");
            config.driveGeometry=DriveGeometry.parse(FieldPackage.map(root.get("drive_geometry")));
        }
        if (root.containsKey("tires")) {
            config.tires = TireDriveConfig.parse((Map<String, Object>) root.get("tires"));
            if (config.drive == null || config.urdf == null) throw new IllegalArgumentException("tires requires differential drive and URDF");
        }
        if(root.containsKey("rotating_wheels")) {
            config.rotatingWheels=RotatingWheelConfig.parse(FieldPackage.map(root.get("rotating_wheels")));
            if(config.drive==null||config.urdf==null||config.tires!=null||config.driveContacts!=null&&config.driveContacts.enabled())
                throw new IllegalArgumentException("rotating_wheels requires URDF and differential drive, without tires or enabled drive_contacts; native contacts own traction");
        }
        if (root.containsKey("intake")) config.intake = MotorIntakeConfig.parse((Map<String, Object>) root.get("intake"));
        if (root.containsKey("flexible_intake")) {
            config.flexibleIntake = FlexibleIntakeConfig.parse((Map<String,Object>)root.get("flexible_intake"));
            if (config.urdf == null || config.intake == null) throw new IllegalArgumentException("flexible_intake requires URDF and motor intake");
        }
        if (root.containsKey("torus_retention")) {
            config.torusRetention=TorusRetentionConfig.parse((Map<String,Object>)root.get("torus_retention"));
            if(config.flexibleIntake==null) throw new IllegalArgumentException("torus_retention requires flexible_intake");
            var seat=config.torusRetention.seat();var min=config.flexibleIntake.containmentMin();var max=config.flexibleIntake.containmentMax();
            for(int i=0;i<3;i++) if(seat.get(i)<min.get(i)||seat.get(i)>max.get(i))
                throw new IllegalArgumentException("Retention seat must be inside containment bounds");
            if(config.torusRetention.exit().x<=max.x) throw new IllegalArgumentException("Retention exit must be ahead of containment bounds");
        }
        if(root.containsKey("game_piece_start_xyz_m")) {
            Object value=root.get("game_piece_start_xyz_m");
            if(!(value instanceof List<?> xyz)||xyz.size()!=3||xyz.stream().anyMatch(x->!(x instanceof Number)))
                throw new IllegalArgumentException("game_piece_start_xyz_m requires three coordinates");
            config.gamePieceStart=ImportedRobotScene.position(xyz.stream().mapToDouble(x->((Number)x).doubleValue()).toArray());
            var p=config.gamePieceStart;
            if(!Float.isFinite(p.x)||!Float.isFinite(p.y)||!Float.isFinite(p.z)||p.y<0)
                throw new IllegalArgumentException("Game-piece start must be finite and above the floor");
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
