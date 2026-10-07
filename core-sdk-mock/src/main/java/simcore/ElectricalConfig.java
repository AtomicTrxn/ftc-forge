package simcore;

import com.qualcomm.robotcore.hardware.HardwareMap;
import physics.BatteryPack;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

/**
 * Tunable power-system, bus and sensor-error settings parsed from {@code sim.config}. Every value
 * is a plain parameter with a validated range and a documented default so it can be adjusted by
 * hand now and fitted from a recording later. Defaults are estimates, not measurements, and
 * keep the pre-existing behavior (the optional models are off until configured).
 *
 * <pre>
 * "battery": {internal_voltage_v, internal_resistance_ohm, capacity_mah, initial_charge_fraction,
 *             empty_voltage_v, curve_shape, empty_resistance_ohm, brownout_voltage_v, brownout_hold_s, brownout_recovery_voltage_v},
 * "loads":   {hub_baseline_a, servo_hold_a, servo_active_a, servo_active_duration_ms},
 * "imu_latency_ms": n, "encoder_latency_ms": n,
 * "bus":     {voltage_latency_ms, current_latency_ms, read_cost_ms, velocity_window_ms},
 * "imu":     {yaw_scale_error, yaw_drift_rad_s, gyro_bias_rad_s, gyro_noise_rad_s, angle_noise_rad, seed, read_cost_ms},
 * "odometry": {latency_ms, linear_scale_error, heading_scale_error, heading_drift_rad_s,
 *              position_noise_m, heading_noise_rad, seed, read_cost_ms},
 * "motor_defaults": {rotor_inertia_kg_m2, static_friction_nm, viscous_friction_nm_s_per_rad,
 *                    thermal_threshold_fraction, thermal_time_constant_s, thermal_max_derate,
 *                    velocity_p_gain, position_p_gain},
 * "motor_overrides": {"motorName": {same keys as motor_defaults}}
 * </pre>
 */
public final class ElectricalConfig {
    public double batteryInternalVoltageV = 12.6;
    public double batteryInternalResistanceOhm = 0.15;
    public double batteryCapacityMah = 0;
    public double initialChargeFraction = 1.0;
    public double emptyVoltageV = 10.5;
    public double curveShape = 1.0;
    public double emptyResistanceOhm = -1;         // -1: resistance does not rise as the pack drains
    public double brownoutVoltageV = 0;
    public double brownoutHoldS = 2.0;
    public double brownoutRecoveryVoltageV = -1;   // -1: brownout voltage + 0.5

    public double hubBaselineA = 0;
    public double servoHoldA = 0;
    public double servoActiveA = 0;
    public long servoActiveDurationMs = 300;

    public long imuLatencyMs = 8;
    public long encoderLatencyMs = SimDcMotorEx.DEFAULT_ENCODER_LATENCY_MS;
    public long voltageLatencyMs = 0;
    public long currentLatencyMs = 0;
    public double readCostMs = 0;

    public long velocityWindowMs = 0;
    public ImuParams imu = ImuParams.IDEAL;
    public OdometryParams odometry = OdometryParams.IDEAL;
    public MotorTuning motorDefaults = MotorTuning.NONE;
    public final Map<String, MotorTuning> motorOverrides = new LinkedHashMap<>();

    /** Top-level configuration keys this class reads; used to lift them out of sim.config or a calibration profile. */
    public static final Set<String> KEYS = Set.of("battery", "loads", "imu_latency_ms", "encoder_latency_ms", "bus", "imu",
        "odometry", "motor_defaults", "motor_overrides");

    private static final Set<String> BATTERY_KEYS = Set.of("internal_voltage_v", "internal_resistance_ohm", "capacity_mah",
        "initial_charge_fraction", "empty_voltage_v", "curve_shape", "empty_resistance_ohm", "brownout_voltage_v", "brownout_hold_s",
        "brownout_recovery_voltage_v");
    private static final Set<String> LOAD_KEYS = Set.of("hub_baseline_a", "servo_hold_a", "servo_active_a", "servo_active_duration_ms");
    private static final Set<String> BUS_KEYS = Set.of("voltage_latency_ms", "current_latency_ms", "read_cost_ms", "velocity_window_ms");
    private static final Set<String> IMU_KEYS = Set.of("yaw_scale_error", "yaw_drift_rad_s", "gyro_bias_rad_s", "gyro_noise_rad_s",
        "angle_noise_rad", "seed", "read_cost_ms");
    private static final Set<String> ODOMETRY_KEYS = Set.of("latency_ms", "linear_scale_error", "heading_scale_error",
        "heading_drift_rad_s", "position_noise_m", "heading_noise_rad", "seed", "read_cost_ms");
    private static final Set<String> MOTOR_KEYS = Set.of("rotor_inertia_kg_m2", "static_friction_nm", "viscous_friction_nm_s_per_rad",
        "thermal_threshold_fraction", "thermal_time_constant_s", "thermal_max_derate", "velocity_p_gain", "position_p_gain");

    @SuppressWarnings("unchecked")
    public static ElectricalConfig parse(Map<String, Object> root) {
        ElectricalConfig c = new ElectricalConfig();
        Map<String, Object> battery = block(root, "battery", BATTERY_KEYS);
        c.batteryInternalVoltageV = num(battery, "internal_voltage_v", c.batteryInternalVoltageV, 6, 18);
        c.batteryInternalResistanceOhm = num(battery, "internal_resistance_ohm", c.batteryInternalResistanceOhm, 0, 2);
        c.batteryCapacityMah = num(battery, "capacity_mah", c.batteryCapacityMah, 0, 100000);
        c.initialChargeFraction = num(battery, "initial_charge_fraction", c.initialChargeFraction, 0, 1);
        c.emptyVoltageV = num(battery, "empty_voltage_v", c.emptyVoltageV, 0, 18);
        c.curveShape = num(battery, "curve_shape", c.curveShape, 0.1, 10);
        c.emptyResistanceOhm = num(battery, "empty_resistance_ohm", c.emptyResistanceOhm, -1, 2);
        c.brownoutVoltageV = num(battery, "brownout_voltage_v", c.brownoutVoltageV, 0, 18);
        c.brownoutHoldS = num(battery, "brownout_hold_s", c.brownoutHoldS, 0, 60);
        c.brownoutRecoveryVoltageV = num(battery, "brownout_recovery_voltage_v", c.brownoutRecoveryVoltageV, -1, 18);

        Map<String, Object> loads = block(root, "loads", LOAD_KEYS);
        c.hubBaselineA = num(loads, "hub_baseline_a", c.hubBaselineA, 0, 20);
        c.servoHoldA = num(loads, "servo_hold_a", c.servoHoldA, 0, 10);
        c.servoActiveA = num(loads, "servo_active_a", c.servoActiveA, 0, 10);
        c.servoActiveDurationMs = (long) num(loads, "servo_active_duration_ms", c.servoActiveDurationMs, 0, 60000);

        c.imuLatencyMs = (long) num(root, "imu_latency_ms", c.imuLatencyMs, 0, 200);
        c.encoderLatencyMs = (long) num(root, "encoder_latency_ms", c.encoderLatencyMs, 0, 200);
        Map<String, Object> bus = block(root, "bus", BUS_KEYS);
        c.voltageLatencyMs = (long) num(bus, "voltage_latency_ms", c.voltageLatencyMs, 0, 200);
        c.currentLatencyMs = (long) num(bus, "current_latency_ms", c.currentLatencyMs, 0, 200);
        c.readCostMs = num(bus, "read_cost_ms", c.readCostMs, 0, 50);
        c.velocityWindowMs = (long) num(bus, "velocity_window_ms", c.velocityWindowMs, 0, 200);

        Map<String, Object> imu = block(root, "imu", IMU_KEYS);
        if (!imu.isEmpty()) {
            double imuSeed = num(imu, "seed", 0, -1e9, 1e9);
            if (imuSeed != (long) imuSeed) throw new IllegalArgumentException("imu seed must be an integer");
            c.imu = new ImuParams(num(imu, "yaw_scale_error", 0, -0.5, 0.5), num(imu, "yaw_drift_rad_s", 0, -0.1, 0.1),
                num(imu, "gyro_bias_rad_s", 0, -0.5, 0.5), num(imu, "gyro_noise_rad_s", 0, 0, 0.5),
                num(imu, "angle_noise_rad", 0, 0, 0.1), (long) imuSeed, num(imu, "read_cost_ms", 0, 0, 50));
        }

        Map<String, Object> odo = block(root, "odometry", ODOMETRY_KEYS);
        if (!odo.isEmpty()) {
            double seed = num(odo, "seed", 0, -1e9, 1e9);
            if (seed != (long) seed) throw new IllegalArgumentException("odometry seed must be an integer");
            c.odometry = new OdometryParams((long) num(odo, "latency_ms", 0, 0, 200),
                num(odo, "linear_scale_error", 0, -0.5, 0.5), num(odo, "heading_scale_error", 0, -0.5, 0.5),
                num(odo, "heading_drift_rad_s", 0, -0.1, 0.1), num(odo, "position_noise_m", 0, 0, 0.05),
                num(odo, "heading_noise_rad", 0, 0, 0.1), (long) seed, num(odo, "read_cost_ms", 0, 0, 50));
        }

        Map<String, Object> defaults = block(root, "motor_defaults", MOTOR_KEYS);
        if (!defaults.isEmpty()) c.motorDefaults = motor(defaults);
        Object overrides = root.get("motor_overrides");
        if (overrides != null) {
            if (!(overrides instanceof Map<?, ?> map)) throw new IllegalArgumentException("motor_overrides must be an object");
            for (var entry : ((Map<String, Object>) map).entrySet()) {
                if (!(entry.getValue() instanceof Map<?, ?>))
                    throw new IllegalArgumentException("motor_overrides." + entry.getKey() + " must be an object");
                Map<String, Object> values = (Map<String, Object>) entry.getValue();
                reject(values, MOTOR_KEYS, "motor_overrides." + entry.getKey());
                c.motorOverrides.put(entry.getKey(), motor(values));
            }
        }
        if (c.emptyVoltageV > c.batteryInternalVoltageV)
            throw new IllegalArgumentException("battery.empty_voltage_v must not exceed internal_voltage_v");
        return c;
    }

    /**
     * Applies the settings to a freshly built hardware map. Call before loading a calibration
     * profile: measured values from a profile take precedence over these.
     */
    public void apply(HardwareMap map) {
        var model = HardwareMapBuilder.getBatteryModel();
        model.vInternal = batteryInternalVoltageV;
        model.rBattery = batteryInternalResistanceOhm;

        BatteryPack pack = HardwareMapBuilder.getBatteryPack();
        pack.capacityAh = batteryCapacityMah / 1000.0;
        pack.emptyVoltageV = emptyVoltageV;
        pack.shape = curveShape;
        pack.emptyResistanceOhm = emptyResistanceOhm;
        pack.brownoutV = brownoutVoltageV;
        pack.brownoutHoldS = brownoutHoldS;
        pack.brownoutRecoveryV = brownoutRecoveryVoltageV >= 0 ? brownoutRecoveryVoltageV : brownoutVoltageV + 0.5;
        pack.setChargeFraction(initialChargeFraction);
        HardwareMapBuilder.configureLoads(hubBaselineA, servoHoldA, servoActiveA, servoActiveDurationMs);

        for (var hub : map.getAll(SimVoltageSensor.class)) {
            hub.setVoltageLatencyMs(voltageLatencyMs);
            hub.setReadCostMs(readCostMs);
        }
        for (var motor : map.getAll(SimDcMotorEx.class)) {
            motor.setEncoderLatencyMs(encoderLatencyMs);
            motor.setCurrentLatencyMs(currentLatencyMs);
            motor.setVelocityWindowMs(velocityWindowMs);
            MotorTuning own = motorOverrides.get(motor.getDeviceName());
            motor.applyTuning(own == null ? motorDefaults : own.over(motorDefaults));
        }
        for (String name : motorOverrides.keySet()) {
            if (map.tryGet(SimDcMotorEx.class, name) == null)
                throw new IllegalArgumentException("motor_overrides names an unknown motor: " + name);
        }
        for (var device : map.getAll(com.qualcomm.robotcore.hardware.HardwareDevice.class)) {
            if (device instanceof SimIMU simImu) {
                simImu.setLatencyMs(imuLatencyMs);
                simImu.configureErrors(imu, map.getNamesOf(device).stream().findFirst().orElse("imu"));
            }
            if (device instanceof PoseSink sink) {
                String name = map.getNamesOf(device).stream().findFirst().orElse("odometry");
                sink.configureOdometry(odometry, name);
            }
        }
    }

    private static MotorTuning motor(Map<String, Object> m) {
        return new MotorTuning(opt(m, "rotor_inertia_kg_m2"), opt(m, "static_friction_nm"), opt(m, "viscous_friction_nm_s_per_rad"),
            opt(m, "thermal_threshold_fraction"), opt(m, "thermal_time_constant_s"), opt(m, "thermal_max_derate"),
            opt(m, "velocity_p_gain"), opt(m, "position_p_gain"));
    }

    private static Double opt(Map<String, Object> m, String key) {
        if (!m.containsKey(key)) return null;
        if (!(m.get(key) instanceof Number n) || !Double.isFinite(n.doubleValue()))
            throw new IllegalArgumentException(key + " must be a finite number");
        return n.doubleValue();
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> block(Map<String, Object> root, String name, Set<String> allowed) {
        Object value = root.get(name);
        if (value == null) return Map.of();
        if (!(value instanceof Map<?, ?>)) throw new IllegalArgumentException(name + " must be an object");
        Map<String, Object> map = (Map<String, Object>) value;
        reject(map, allowed, name);
        return map;
    }

    private static void reject(Map<String, Object> map, Set<String> allowed, String where) {
        for (String key : map.keySet())
            if (!allowed.contains(key)) throw new IllegalArgumentException("Unknown " + where + " setting: " + key);
    }

    private static double num(Map<String, Object> map, String key, double fallback, double low, double high) {
        if (!map.containsKey(key)) return fallback;
        if (!(map.get(key) instanceof Number n)) throw new IllegalArgumentException(key + " must be a number");
        double v = n.doubleValue();
        if (!(v >= low && v <= high)) throw new IllegalArgumentException(key + " must be between " + low + " and " + high);
        return v;
    }
}
