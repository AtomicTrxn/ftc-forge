package physics;

import org.junit.jupiter.api.Test;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

class BatteryModelTest {

    private final MotorSpec spec = new MotorSpec("goBILDA-5203-19.2", 19.2, 2.383, 9.2, 32.67, 12.0, 537.7);

    @Test
    void noLoad_producesInternalVoltageExactly() {
        BatteryModel battery = new BatteryModel(12.6, 0.15);
        double v = battery.solveBatteryVoltage(List.of(new BatteryModel.MotorState(0.0, spec, 0.0)));
        assertEquals(12.6, v, 1e-9);
    }

    @Test
    void stalledMotorAtFullPower_sagsVoltageBelowInternal() {
        BatteryModel battery = new BatteryModel(12.6, 0.15);
        // Full power, stalled (omega=0): current draw = iStall = 9.2A.
        double v = battery.solveBatteryVoltage(List.of(new BatteryModel.MotorState(1.0, spec, 0.0)));
        assertTrue(v < 12.6, "expected sag under load, got " + v);
        // Manually solve the single-motor closed form to cross-check the general formula:
        // A = 1^2 * 9.2/12.0 = 0.76667; B = 1*9.2*0 = 0 -> Vb = 12.6 / (1 + 0.15*0.76667)
        double expected = 12.6 / (1 + 0.15 * (9.2 / 12.0));
        assertEquals(expected, v, 1e-9);
    }

    @Test
    void multipleMotors_currentDrawIsAdditive() {
        BatteryModel battery = new BatteryModel(12.6, 0.15);
        double vOneMotor = battery.solveBatteryVoltage(List.of(new BatteryModel.MotorState(1.0, spec, 0.0)));
        double vFourMotors = battery.solveBatteryVoltage(List.of(
            new BatteryModel.MotorState(1.0, spec, 0.0), new BatteryModel.MotorState(1.0, spec, 0.0),
            new BatteryModel.MotorState(1.0, spec, 0.0), new BatteryModel.MotorState(1.0, spec, 0.0)));
        assertTrue(vFourMotors < vOneMotor, "four loaded motors should sag more than one");
    }
}
