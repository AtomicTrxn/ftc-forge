package simcore;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/** Combines a parsed robot-config XML (device names/ports) with a preset (motor SKU/ratio) into a live HardwareMap. */
public class HardwareMapBuilder {

    public static HardwareMap build(RobotConfigXml xml, PresetRobotConfig preset) {
        HardwareMap map = new HardwareMap();
        for (RobotConfigXml.DeviceEntry entry : xml.devices) {
            RobotConfigXml.DeviceType type = RobotConfigXml.resolveType(entry.tag);
            switch (type) {
                case MOTOR: {
                    SimDcMotorEx.MotorSpec spec = preset.motors.get(entry.name);
                    if (spec == null) {
                        spec = new SimDcMotorEx.MotorSpec("unknown", 1.0, 2000.0);
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

    public static void tickMotors(HardwareMap map) {
        List<com.qualcomm.robotcore.hardware.DcMotor> motors = map.getAll(com.qualcomm.robotcore.hardware.DcMotor.class);
        for (com.qualcomm.robotcore.hardware.DcMotor m : motors) {
            if (m instanceof SimDcMotorEx) {
                ((SimDcMotorEx) m).tick();
            }
        }
    }
}
