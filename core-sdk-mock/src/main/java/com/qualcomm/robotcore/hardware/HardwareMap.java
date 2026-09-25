package com.qualcomm.robotcore.hardware;

import java.util.*;

/**
 * Mock of com.qualcomm.robotcore.hardware.HardwareMap.
 *
 * Per R1/R3: behaviorally a typed, string-keyed registry. Lookup is type-aware, not just
 * name-keyed (get(DcMotorEx.class, n) and get(DcMotor.class, n) for the same device resolve
 * to the same object; a request for an incompatible type throws, matching real SDK behavior
 * so team try/catch code behaves the same as on real hardware).
 */
public class HardwareMap {

    private final Map<String, HardwareDevice> devicesByName = new LinkedHashMap<>();

    public void register(String name, HardwareDevice device) {
        devicesByName.put(name, device);
    }

    @SuppressWarnings("unchecked")
    public <T> T get(Class<? extends T> type, String deviceName) {
        HardwareDevice device = devicesByName.get(deviceName);
        if (device == null) {
            throw new IllegalArgumentException(
                "Unable to find a hardware device with name \"" + deviceName + "\"");
        }
        if (!type.isInstance(device)) {
            throw new IllegalArgumentException(
                "Hardware device with name \"" + deviceName + "\" is of type "
                    + device.getClass().getSimpleName() + ", not " + type.getSimpleName());
        }
        return (T) device;
    }

    public HardwareDevice get(String deviceName) {
        return get(HardwareDevice.class, deviceName);
    }

    public <T> T tryGet(Class<? extends T> type, String deviceName) {
        try {
            return get(type, deviceName);
        } catch (IllegalArgumentException e) {
            return null;
        }
    }

    @SuppressWarnings("unchecked")
    public <T> List<T> getAll(Class<? extends T> type) {
        List<T> result = new ArrayList<>();
        for (HardwareDevice d : devicesByName.values()) {
            if (type.isInstance(d)) {
                result.add((T) d);
            }
        }
        return result;
    }

    public Set<String> getNamesOf(HardwareDevice device) {
        Set<String> names = new LinkedHashSet<>();
        for (Map.Entry<String, HardwareDevice> e : devicesByName.entrySet()) {
            if (e.getValue() == device) {
                names.add(e.getKey());
            }
        }
        return names;
    }

    /** Minimal DeviceMapping-style typed iteration, matching the real hardwareMap.voltageSensor idiom. */
    public class DeviceMapping<T extends HardwareDevice> implements Iterable<T> {
        private final Class<T> type;
        DeviceMapping(Class<T> type) { this.type = type; }
        @Override public Iterator<T> iterator() { return getAll(type).iterator(); }
    }

    public final DeviceMapping<VoltageSensor> voltageSensor = new DeviceMapping<>(VoltageSensor.class);
    public final DeviceMapping<DcMotor> dcMotor = new DeviceMapping<>(DcMotor.class);
    public final DeviceMapping<Servo> servo = new DeviceMapping<>(Servo.class);
}
