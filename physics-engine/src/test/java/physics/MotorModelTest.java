package physics;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/** Validates against R4's own Validation Notes reference points, including the test that
 *  specifically distinguishes the corrected equation from the plan's original (wrong) one. */
class MotorModelTest {

    private static final double EPS = 1e-9;
    // goBILDA 5203 series, 50.9:1 ratio (R4's reference fixture).
    private final MotorSpec spec = new MotorSpec("goBILDA-5203-50.9", 50.9, 6.71, 9.2, 12.25, 12.0, 1425.1);
    private final MotorModel model = new MotorModel(); // no friction/thermal for these reference-point checks

    @Test
    void stallAtNominalVoltage_producesRatedStallTorque() {
        assertEquals(6.71, model.torque(spec, 0, 12.0), EPS);
    }

    @Test
    void noLoadSpeedAtNominalVoltage_producesZeroTorque() {
        assertEquals(0.0, model.torque(spec, 12.25, 12.0), 1e-6);
    }

    @Test
    void halfVoltage_halfSpeed_producesZeroTorque_theBugDistinguishingTest() {
        // The plan's original (wrong) equation gives tau_stall/2 here, not 0 -- this is the
        // test that would have caught the original bug (per R4's Validation Notes).
        assertEquals(0.0, model.torque(spec, 0.5 * spec.omegaNoLoadRadS, 0.5 * spec.vNominal), 1e-6);
    }

    @Test
    void halfVoltage_zeroSpeed_producesHalfStallTorque() {
        // Both the original and corrected equations agree here -- linearity in V check only.
        assertEquals(3.355, model.torque(spec, 0, 6.0), 1e-6);
    }

    @Test
    void brakeAtNonZeroSpeed_producesReverseBrakingTorque() {
        // V=0 (brake): the original equation could never model this (always returned 0).
        double t = model.torque(spec, spec.omegaNoLoadRadS / 2, 0.0);
        assertTrue(t < 0, "expected negative (braking) torque, got " + t);
    }

    @Test
    void currentEquation_matchesStallAndNoLoadPoints() {
        assertEquals(9.2, model.current(spec, 0, 12.0), EPS);
        assertEquals(0.0, model.current(spec, 12.25, 12.0), 1e-6);
    }

    @Test
    void frictionAddition_reducesAvailableTorqueButNotCurrent() {
        MotorModel withFriction = new MotorModel();
        withFriction.tauStaticNm = 0.2;
        withFriction.viscousBNms = 0.01;
        double idealTorque = model.torque(spec, 5.0, 12.0);
        double frictionTorque = withFriction.torque(spec, 5.0, 12.0);
        assertTrue(frictionTorque < idealTorque, "friction should reduce available torque");
        // Current is a function of the ideal EMF relation only -- friction doesn't change it.
        assertEquals(model.current(spec, 5.0, 12.0), withFriction.current(spec, 5.0, 12.0), EPS);
    }

    // --- winding thermal model (opt-in) ---

    private MotorModel winding() {
        MotorModel m = new MotorModel();
        m.thermalCapacityJPerK = 40;
        m.thermalResistanceKPerW = 6;
        m.ambientC = 25;
        m.resistanceTempcoPerK = 0;          // keep the analytic check exact
        return m;
    }

    @Test
    void windingModelIsOffByDefaultAndLeavesTorqueAndTemperatureAlone() {
        assertFalse(model.windingModelEnabled());
        MotorModel fresh = new MotorModel();
        fresh.tickThermal(spec, 9.2, 100);          // the old duty-cycle heuristic still runs; the winding temperature does not move
        assertEquals(fresh.ambientC, fresh.windingTempC());
        assertEquals(6.71, model.torque(spec, 0, 12.0), EPS);
    }

    @Test
    void windingHeatsAlongTheFirstOrderCurveForAConstantCurrent() {
        MotorModel m = winding();
        double current = 3.0, r = spec.vNominal / spec.iStallAmps;
        double steady = 25 + current * current * r * 6, tau = 6 * 40;
        for (int i = 0; i < 100; i++) m.tickThermal(spec, current, 1.0);
        assertEquals(steady + (25 - steady) * Math.exp(-100 / tau), m.windingTempC(), 1e-9);
        for (int i = 0; i < 5000; i++) m.tickThermal(spec, current, 1.0);
        assertEquals(steady, m.windingTempC(), 1e-6);
        for (int i = 0; i < 5000; i++) m.tickThermal(spec, 0, 1.0);
        assertEquals(25, m.windingTempC(), 1e-6, "cools back to ambient");
    }

    @Test
    void stepSizeDoesNotChangeTheThermalResult() {
        MotorModel coarse = winding(), fine = winding();
        coarse.tickThermal(spec, 4.0, 10.0);
        for (int i = 0; i < 1000; i++) fine.tickThermal(spec, 4.0, 0.01);
        assertEquals(coarse.windingTempC(), fine.windingTempC(), 1e-9);
    }

    @Test
    void hotWindingDeratesTorqueBetweenTheConfiguredTemperatures() {
        MotorModel m = winding();
        m.derateStartC = 40;
        m.derateFullC = 60;
        m.thermalMaxDerate = 0.4;
        double cold = m.torque(spec, 0, 12.0);
        // Hold the winding at ~50 C: steady = 25 + I^2 R Rth.
        double r = spec.vNominal / spec.iStallAmps, current = Math.sqrt(25 / (r * 6));
        for (int i = 0; i < 5000; i++) m.tickThermal(spec, current, 1.0);
        assertEquals(50, m.windingTempC(), 1e-3);
        assertEquals(cold * (1 - 0.4 * 0.5), m.torque(spec, 0, 12.0), 1e-6);
    }

    @Test
    void resistanceRisesWithTemperatureSoTorqueAndCurrentFall() {
        MotorModel m = winding();
        m.resistanceTempcoPerK = 0.004;
        m.derateStartC = 1000;
        m.derateFullC = 2000;
        double coldTorque = m.torque(spec, 0, 12.0), coldCurrent = m.current(spec, 0, 12.0);
        for (int i = 0; i < 3000; i++) m.tickThermal(spec, 2.0, 1.0);
        double scale = 1 / (1 + 0.004 * (m.windingTempC() - 25));
        assertTrue(scale < 0.95);
        assertEquals(coldTorque * scale, m.torque(spec, 0, 12.0), 1e-9);
        assertEquals(coldCurrent * scale, m.current(spec, 0, 12.0), 1e-9);
    }
}
