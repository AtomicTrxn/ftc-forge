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
}
