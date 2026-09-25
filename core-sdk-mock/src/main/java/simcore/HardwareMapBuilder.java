package simcore;

import com.qualcomm.robotcore.hardware.HardwareMap;
import physics.BatteryModel;
import physics.MotorSpec;

import java.util.ArrayList;
import java.util.List;

/** Combines a parsed robot-config XML (device names/ports) with a preset (motor SKU/ratio) into a live HardwareMap. */
public class HardwareMapBuilder {

    // Shared battery model + clock state for the whole robot -- one battery, one clock, per R4.
    private static final BatteryModel BATTERY = new BatteryModel(12.6, 0.15); // per R4's revised default
    private static long lastTickNanos = 0;
    private static long simTimeMs = 0;

    public static HardwareMap build(RobotConfigXml xml, PresetRobotConfig preset) {
        HardwareMap map = new HardwareMap();
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
                    map.register(entry.name, new SimDcMotorEx(entry.name, spec));
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
                default:
                    System.out.println("[WARN] Unrecognized device tag <" + entry.tag + "> for \""
                        + entry.name + "\" -- skipping. See RobotConfigXml.resolveType.");
            }
        }
        // Per R3: each hub is itself registered as a VoltageSensor.
        for (String hubName : xml.hubNames) {
            map.register(hubName, new SimVoltageSensor(hubName));
        }
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

        List<SimDcMotorEx> motors = new ArrayList<>();
        for (var m : map.getAll(com.qualcomm.robotcore.hardware.DcMotor.class)) {
            if (m instanceof SimDcMotorEx) motors.add((SimDcMotorEx) m);
        }
        if (motors.isEmpty() || dtSeconds <= 0) return;

        List<BatteryModel.MotorState> states = new ArrayList<>();
        for (SimDcMotorEx m : motors) {
            states.add(new BatteryModel.MotorState(m.commandedPower(), m.getSpec(), m.getOmegaRadS()));
        }
        double batteryVoltage = BATTERY.solveBatteryVoltage(states);

        for (SimDcMotorEx m : motors) {
            m.integrate(batteryVoltage, dtSeconds, simTimeMs);
        }
        for (var v : map.getAll(SimVoltageSensor.class)) {
            v.setVoltage(batteryVoltage);
        }
    }

    public static BatteryModel getBatteryModel() { return BATTERY; }
}
