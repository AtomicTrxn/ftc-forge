package simrunner;
import java.util.Map;
import java.util.Set;
/** Opt-in native differential wheels; no aggregate or brush traction is applied. */
record RotatingWheelConfig(double reflectedMotorInertiaKgM2) {
    RotatingWheelConfig {
        if(!Double.isFinite(reflectedMotorInertiaKgM2)||reflectedMotorInertiaKgM2<=0||reflectedMotorInertiaKgM2>1)
            throw new IllegalArgumentException("rotating_wheels reflected motor inertia must be >0..1 kg*m^2");
    }
    static RotatingWheelConfig parse(Map<String,Object> values) {
        if(!values.keySet().equals(Set.of("reflected_motor_inertia_kg_m2")))throw new IllegalArgumentException("rotating_wheels requires reflected_motor_inertia_kg_m2 only");
        return new RotatingWheelConfig(FieldPackage.num(values,"reflected_motor_inertia_kg_m2"));
    }
}
