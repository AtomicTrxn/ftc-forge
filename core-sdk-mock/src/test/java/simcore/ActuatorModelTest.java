package simcore;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.junit.jupiter.api.Test;
import physics.MotorSpec;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Transmission efficiency, backlash and the extra velocity-loop gains. */
class ActuatorModelTest {
    private static final MotorSpec SPEC = new MotorSpec("t", 19.2, 2.4, 9.2, 32.7, 12, 1000);

    private static SimDcMotorEx motor(String tuningJson) {
        SimDcMotorEx motor = new SimDcMotorEx("m", SPEC);
        if (tuningJson != null) {
            var root = MiniJson.parseObject("{\"motor_defaults\":" + tuningJson + "}");
            motor.applyTuning(ElectricalConfig.parse(root).motorDefaults);
        }
        return motor;
    }

    @Test void transmissionEfficiencyScalesDrivingTorqueAndAmplifiesBrakingTorque() {
        SimDcMotorEx lossy = motor("{\"transmission_efficiency\":0.8}");
        SimDcMotorEx ideal = motor(null);
        for (SimDcMotorEx m : List.of(lossy, ideal)) {
            m.useExternalShaft();
            m.setPower(1);
            m.syncExternalShaft(0, 0);
            m.integrate(12, 0.01, 10);
        }
        assertEquals(2.4, ideal.externalShaftTorque(), 1e-3);       // the thermal heuristic has derated a hair after one tick
        assertEquals(ideal.externalShaftTorque() * 0.8, lossy.externalShaftTorque(), 1e-9, "driving from rest delivers 80%");
        lossy.syncExternalShaft(0, -10);
        ideal.syncExternalShaft(0, -10);
        // Shaft turning against the torque: the load sees the braking torque divided by the efficiency.
        assertEquals(ideal.externalShaftTorque() / 0.8, lossy.externalShaftTorque(), 1e-9);
        lossy.syncExternalShaft(0, 10);
        ideal.syncExternalShaft(0, 10);
        assertEquals(ideal.externalShaftTorque() * 0.8, lossy.externalShaftTorque(), 1e-9);
    }

    @Test void backlashMakesTheEncoderTrailTheLoadAndHoldThroughAReversal() {
        SimDcMotorEx m = motor("{\"backlash_motor_rad\":0.2}");
        m.useExternalShaft();
        double perRad = 1000 / (2 * Math.PI);
        long t = 0;
        m.syncExternalShaft(0, 0);
        m.integrate(12, 0.01, t += 10);
        m.syncExternalShaft(1.0, 1);
        m.integrate(12, 0.01, t += 10);
        assertEquals((1.0 - 0.1) * perRad, m.getRawTicks(), 1e-6, "trails by half the gap going forward");
        m.syncExternalShaft(0.95, -1);                           // reverses inside the gap
        m.integrate(12, 0.01, t += 10);
        assertEquals((1.0 - 0.1) * perRad, m.getRawTicks(), 1e-6, "encoder holds while the load crosses the play");
        m.syncExternalShaft(0.7, -1);                            // out the other side
        m.integrate(12, 0.01, t += 10);
        assertEquals((0.7 + 0.1) * perRad, m.getRawTicks(), 1e-6, "now trails from the other side");
    }

    @Test void zeroBacklashIsTransparent() {
        SimDcMotorEx m = motor(null);
        m.useExternalShaft();
        m.syncExternalShaft(1.234, 0);
        m.integrate(12, 0.01, 10);
        assertEquals(1.234 * 1000 / (2 * Math.PI), m.getRawTicks(), 1e-6);
    }

    /** Settled speed error of a loaded motor in RUN_USING_ENCODER at a target speed. */
    private static double velocityError(String tuning) {
        SimDcMotorEx m = motor(tuning);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double target = 0.5 * SPEC.omegaNoLoadRadS;
        m.setVelocity(target);
        for (int i = 1; i <= 2000; i++) m.integrate(12, 0.01, i * 10L);
        return Math.abs(m.getOmegaRadS() - target) / target;
    }

    @Test void integralGainRemovesTheSteadyStateErrorOfAProportionalOnlyLoop() {
        String load = "\"viscous_friction_nm_s_per_rad\":0.05";
        double proportionalOnly = velocityError("{" + load + "}");
        double withIntegral = velocityError("{" + load + ",\"velocity_i_gain\":2}");
        assertTrue(proportionalOnly > 0.03, "P-only leaves an error: " + proportionalOnly);
        assertTrue(withIntegral < 0.005, "I removes it: " + withIntegral);
    }

    @Test void derivativeGainStaysFiniteAndIsClearedOutsideTheVelocityMode() {
        SimDcMotorEx m = motor("{\"velocity_d_gain\":0.01,\"velocity_i_gain\":1}");
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.setVelocity(10);
        for (int i = 1; i <= 100; i++) m.integrate(12, 0.01, i * 10L);
        assertTrue(Double.isFinite(m.commandedPower()));
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setPower(0.3);
        m.integrate(12, 0.01, 2000);
        assertEquals(0.3, m.commandedPower(), 1e-12, "no loop terms outside RUN_USING_ENCODER");
    }

    @Test void newKeysAreValidated() {
        for (String bad : List.of("{\"transmission_efficiency\":0}", "{\"transmission_efficiency\":1.5}",
                "{\"backlash_motor_rad\":-1}", "{\"backlash_motor_rad\":10}", "{\"velocity_i_gain\":-1}", "{\"velocity_d_gain\":5000}"))
            assertThrows(IllegalArgumentException.class, () -> ElectricalConfig.parse(MiniJson.parseObject("{\"motor_defaults\":" + bad + "}")), bad);
        assertThrows(IllegalArgumentException.class, () -> ElectricalConfig.parse(MiniJson.parseObject("{\"battery\":{\"capacity_temp_coeff_per_k\":1}}")));
    }

    @Test void touchSensorReadsBlockForTheConfiguredCost() {
        SimTouchSensor touch = new SimTouchSensor("t");
        touch.setReadCostMs(5);
        long start = System.nanoTime();
        for (int i = 0; i < 6; i++) touch.isPressed();
        assertTrue((System.nanoTime() - start) / 1e6 >= 28);
        assertThrows(IllegalArgumentException.class, () -> touch.setReadCostMs(-1));
    }
}
