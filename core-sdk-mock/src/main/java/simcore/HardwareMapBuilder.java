package simcore;

import com.qualcomm.robotcore.hardware.HardwareMap;
import physics.BatteryModel;
import physics.BatteryPack;
import physics.MotorSpec;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.List;

/** Combines a parsed robot-config XML (device names/ports) with a preset (motor SKU/ratio) into a live HardwareMap. */
public class HardwareMapBuilder {

    // Shared battery model + clock state for the whole robot -- one battery, one clock, per R4.
    private static final BatteryModel BATTERY = new BatteryModel(12.6, 0.15); // per R4's revised default
    private static final BatteryPack PACK = new BatteryPack();
    private static double hubBaselineA, servoHoldA, servoActiveA;
    private static long servoActiveMs = 300;
    private static long lastTickNanos = 0;
    private static long simTimeMs = 0;

    public static synchronized HardwareMap build(RobotConfigXml xml, PresetRobotConfig preset) {
        // A new run starts from defaults; an optional calibration profile is applied afterward.
        BATTERY.vInternal = 12.6;
        BATTERY.rBattery = 0.15;
        resetPack();
        configureLoads(0, 0, 0, 300);
        lastTickNanos = 0;
        simTimeMs = 0;
        HardwareMap map = new HardwareMap();
        Map<String, SimVoltageSensor> hubs = new LinkedHashMap<>();
        for (String hubName : xml.hubNames) hubs.put(hubName, new SimVoltageSensor(hubName));
        for (RobotConfigXml.DeviceEntry entry : xml.devices) {
            RobotConfigXml.DeviceType type = RobotConfigXml.resolveType(entry.tag);
            switch (type) {
                case MOTOR: {
                    MotorSpec spec = preset.motors.get(entry.name);
                    if (spec == null) {
                        spec = new MotorSpec("unknown", 1.0, 2.0, 9.2, 30.0, 12.0, 500.0);
                        System.out.println("[WARN] No preset motor spec for \"" + entry.name
                            + "\" -- using a generic placeholder. Add it to the preset's motors block.");
                    }
                    SimDcMotorEx motor = new SimDcMotorEx(entry.name, spec);
                    SimVoltageSensor hub = hubs.get(entry.hub);
                    if (hub != null) motor.attachHub(hub);
                    map.register(entry.name, motor);
                    break;
                }
                case SERVO:
                    map.register(entry.name, new SimServo(entry.name));
                    break;
                case CR_SERVO:
                    map.register(entry.name, new SimCRServo(entry.name));
                    break;
                case IMU:
                    map.register(entry.name, new SimIMU(entry.name));
                    break;
                case DISTANCE_SENSOR:
                    map.register(entry.name, new SimDistanceSensor(entry.name));
                    break;
                case COLOR_SENSOR:
                    map.register(entry.name, new SimColorSensor(entry.name));
                    break;
                case TOUCH_SENSOR:
                    map.register(entry.name, new SimTouchSensor(entry.name));
                    break;
                case CAMERA:
                    map.register(entry.name, new org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName(entry.name));
                    break;
                case PINPOINT:
                    map.register(entry.name, new com.qualcomm.hardware.gobilda.GoBildaPinpointDriver(entry.name));
                    break;
                case OTOS:
                    map.register(entry.name, new com.qualcomm.hardware.sparkfun.SparkFunOTOS(entry.name));
                    break;
                default:
                    System.out.println("[WARN] Unrecognized device tag <" + entry.tag + "> for \""
                        + entry.name + "\" -- skipping. See RobotConfigXml.resolveType.");
            }
        }
        // Per R3: each hub is itself registered as a VoltageSensor.
        for (var hub : hubs.entrySet()) map.register(hub.getKey(), hub.getValue());
        return map;
    }

    /**
     * Advances all motors by one tick, sharing a single battery voltage across them per
     * R4's closed-form multi-motor model -- this is why motors can no longer tick
     * independently (Phase 1's design): the whole point of the battery model is that one
     * motor's draw affects every other motor's effective voltage in the same tick.
     */
    public static synchronized void tickMotors(HardwareMap map) {
        long now = System.nanoTime();
        if (lastTickNanos == 0) lastTickNanos = now;
        double dtSeconds = Math.min(0.1, (now - lastTickNanos) / 1_000_000_000.0); // clamp against GC pauses/scheduling hiccups
        lastTickNanos = now;
        simTimeMs += Math.round(dtSeconds * 1000);

        tickMotors(map, dtSeconds, simTimeMs);
    }

    /** External fixed-step clock. Do not combine with the wall-clock ticker in one run. */
    public static synchronized void tickMotors(HardwareMap map, double dtSeconds, long timeMs) {
        if (!Double.isFinite(dtSeconds) || dtSeconds < 0 || dtSeconds > .1 || timeMs < 0)
            throw new IllegalArgumentException("Motor tick requires finite dt 0..0.1s and nonnegative time");
        List<SimDcMotorEx> motors = new ArrayList<>();
        for (var m : map.getAll(com.qualcomm.robotcore.hardware.DcMotor.class)) {
            if (m instanceof SimDcMotorEx) motors.add((SimDcMotorEx) m);
        }
        if (motors.isEmpty() || dtSeconds <= 0) return;

        boolean brownedOut = PACK.isBrownedOut();
        double openCircuit = PACK.openCircuitV(BATTERY.vInternal);
        double extraLoadAmps = hubBaselineA;
        for (var device : map.getAll(com.qualcomm.robotcore.hardware.HardwareDevice.class)) {
            if (device instanceof ServoLoad servo) extraLoadAmps += servo.drawAmps(timeMs, servoHoldA, servoActiveA, servoActiveMs);
        }

        List<BatteryModel.MotorState> states = new ArrayList<>();
        for (SimDcMotorEx m : motors) {
            m.setPowerCut(brownedOut);
            states.add(new BatteryModel.MotorState(m.signedCommandedPower(), m.getSpec(), m.getOmegaRadS()));
        }
        double resistance = PACK.resistanceOhm(BATTERY.rBattery);
        double batteryVoltage = BATTERY.solveBatteryVoltage(states, extraLoadAmps, openCircuit, resistance);

        for (SimDcMotorEx m : motors) {
            m.integrate(batteryVoltage, dtSeconds, timeMs);
        }
        PACK.consume(BATTERY.batteryCurrent(states, batteryVoltage) + extraLoadAmps, dtSeconds);
        PACK.updateBrownout(batteryVoltage, openCircuit, timeMs / 1000.0);
        for (var v : map.getAll(SimVoltageSensor.class)) {
            v.publishVoltage(timeMs, batteryVoltage);
        }
    }

    private static void resetPack() {
        PACK.capacityAh = 0;
        PACK.emptyVoltageV = 10.5;
        PACK.shape = 1.0;
        PACK.brownoutV = 0;
        PACK.brownoutHoldS = 2.0;
        PACK.brownoutRecoveryV = 0;
        PACK.emptyResistanceOhm = -1;
        PACK.setChargeFraction(1.0);
    }

    /** Non-motor battery current: a constant hub draw plus per-servo hold and active currents. */
    static synchronized void configureLoads(double hubBaseline, double servoHold, double servoActive, long activeMs) {
        hubBaselineA = hubBaseline;
        servoHoldA = servoHold;
        servoActiveA = servoActive;
        servoActiveMs = activeMs;
    }

    public static BatteryPack getBatteryPack() { return PACK; }

    public static BatteryModel getBatteryModel() { return BATTERY; }
}
