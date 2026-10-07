package simcore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class ElectricalConfigTest {
    @TempDir Path dir;

    private static ElectricalConfig parse(String json) { return ElectricalConfig.parse(MiniJson.parseObject(json)); }

    private HardwareMap robot() throws Exception {
        Path xml = dir.resolve("robot.xml");
        Files.writeString(xml, "<Robot><LynxUsbDevice name=\"u\"><LynxModule name=\"Hub\">"
            + "<goBILDA5202SeriesMotor name=\"left\" port=\"0\"/><goBILDA5202SeriesMotor name=\"right\" port=\"1\"/>"
            + "<goBILDAPinpoint name=\"pp\" port=\"1\"/></LynxModule></LynxUsbDevice></Robot>");
        Path preset = dir.resolve("preset.json");
        String spec = "{\"sku\":\"t\",\"ratio\":19.2,\"tauStallNm\":2.4,\"iStallAmps\":9.2,\"omegaNoLoadRadS\":32.7,\"vNominal\":12,\"encoderCountsPerRev\":537.7}";
        Files.writeString(preset, "{\"name\":\"p\",\"motors\":{\"left\":" + spec + ",\"right\":" + spec + "}}");
        return HardwareMapBuilder.build(RobotConfigXml.parse(xml.toFile()), PresetRobotConfig.load(preset));
    }

    @Test void emptyConfigKeepsDocumentedDefaults() {
        ElectricalConfig c = parse("{}");
        assertEquals(12.6, c.batteryInternalVoltageV);
        assertEquals(0.15, c.batteryInternalResistanceOhm);
        assertEquals(0, c.batteryCapacityMah);
        assertEquals(0, c.brownoutVoltageV);
        assertEquals(8, c.encoderLatencyMs);
        assertEquals(0, c.readCostMs);
        assertSame(OdometryParams.IDEAL, c.odometry);
        assertSame(MotorTuning.NONE, c.motorDefaults);
    }

    @Test void everyDocumentedKeyIsParsed() {
        ElectricalConfig c = parse("{\"battery\":{\"internal_voltage_v\":13,\"internal_resistance_ohm\":0.1,\"capacity_mah\":3000,"
            + "\"initial_charge_fraction\":0.8,\"empty_voltage_v\":10,\"curve_shape\":1.5,\"brownout_voltage_v\":7,"
            + "\"brownout_hold_s\":3,\"brownout_recovery_voltage_v\":9},"
            + "\"loads\":{\"hub_baseline_a\":0.8,\"servo_hold_a\":0.2,\"servo_active_a\":1.1,\"servo_active_duration_ms\":250},"
            + "\"encoder_latency_ms\":12,\"bus\":{\"voltage_latency_ms\":5,\"current_latency_ms\":6,\"read_cost_ms\":2.5},"
            + "\"odometry\":{\"latency_ms\":4,\"linear_scale_error\":0.01,\"heading_scale_error\":-0.02,\"heading_drift_rad_s\":0.001,"
            + "\"position_noise_m\":0.001,\"heading_noise_rad\":0.002,\"seed\":7},"
            + "\"motor_defaults\":{\"rotor_inertia_kg_m2\":0.01,\"viscous_friction_nm_s_per_rad\":0.02},"
            + "\"motor_overrides\":{\"left\":{\"static_friction_nm\":0.3,\"velocity_p_gain\":2.5}}}");
        assertEquals(13, c.batteryInternalVoltageV);
        assertEquals(3000, c.batteryCapacityMah);
        assertEquals(0.8, c.initialChargeFraction);
        assertEquals(9, c.brownoutRecoveryVoltageV);
        assertEquals(1.1, c.servoActiveA);
        assertEquals(250, c.servoActiveDurationMs);
        assertEquals(12, c.encoderLatencyMs);
        assertEquals(6, c.currentLatencyMs);
        assertEquals(2.5, c.readCostMs);
        assertEquals(4, c.odometry.latencyMs);
        assertEquals(7, c.odometry.seed);
        assertEquals(0.01, c.motorDefaults.rotorInertiaKgM2());
        assertEquals(0.3, c.motorOverrides.get("left").staticFrictionNm());
    }

    @Test void sensorErrorBlocksParseAndRejectBadValues() {
        ElectricalConfig c = parse("{\"imu\":{\"yaw_scale_error\":0.01,\"yaw_drift_rad_s\":0.0001,\"gyro_bias_rad_s\":0.001,"
            + "\"gyro_noise_rad_s\":0.002,\"angle_noise_rad\":0.001,\"seed\":3,\"read_cost_ms\":2},"
            + "\"bus\":{\"velocity_window_ms\":50},\"odometry\":{\"read_cost_ms\":1.5},\"battery\":{\"empty_resistance_ohm\":0.4}}");
        assertEquals(0.01, c.imu.yawScaleError);
        assertEquals(2, c.imu.readCostMs);
        assertEquals(50, c.velocityWindowMs);
        assertEquals(1.5, c.odometry.readCostMs);
        assertEquals(0.4, c.emptyResistanceOhm);
        for (String bad : List.of("{\"imu\":{\"bias\":1}}", "{\"imu\":{\"gyro_noise_rad_s\":5}}", "{\"imu\":{\"seed\":0.5}}",
                "{\"bus\":{\"velocity_window_ms\":500}}", "{\"odometry\":{\"read_cost_ms\":99}}", "{\"battery\":{\"empty_resistance_ohm\":9}}"))
            assertThrows(IllegalArgumentException.class, () -> parse(bad), bad);
    }

    @Test void unknownKeysBadTypesAndOutOfRangeValuesAreRejected() {
        for (String bad : List.of(
                "{\"battery\":{\"capacity\":1}}", "{\"loads\":{\"x\":1}}", "{\"bus\":{\"y\":1}}", "{\"odometry\":{\"z\":1}}",
                "{\"motor_defaults\":{\"mass\":1}}", "{\"motor_overrides\":{\"left\":{\"mass\":1}}}", "{\"motor_overrides\":{\"left\":5}}",
                "{\"battery\":[]}", "{\"battery\":{\"capacity_mah\":\"big\"}}", "{\"battery\":{\"capacity_mah\":-1}}",
                "{\"battery\":{\"initial_charge_fraction\":1.5}}", "{\"battery\":{\"empty_voltage_v\":14}}",
                "{\"loads\":{\"hub_baseline_a\":-1}}", "{\"bus\":{\"read_cost_ms\":-1}}", "{\"bus\":{\"voltage_latency_ms\":500}}",
                "{\"odometry\":{\"seed\":1.5}}", "{\"odometry\":{\"position_noise_m\":1}}",
                "{\"motor_defaults\":{\"rotor_inertia_kg_m2\":0}}", "{\"motor_defaults\":{\"thermal_max_derate\":2}}")) {
            assertThrows(IllegalArgumentException.class, () -> parse(bad), bad);
        }
    }

    @Test void overridesBeatDefaultsAndUnspecifiedFieldsFallThrough() throws Exception {
        HardwareMap map = robot();
        parse("{\"motor_defaults\":{\"rotor_inertia_kg_m2\":0.01,\"thermal_max_derate\":0.2},"
            + "\"motor_overrides\":{\"left\":{\"rotor_inertia_kg_m2\":0.05}}}").apply(map);
        assertEquals(0.05, map.get(SimDcMotorEx.class, "left").getRotorInertiaKgM2());
        assertEquals(0.01, map.get(SimDcMotorEx.class, "right").getRotorInertiaKgM2());
    }

    @Test void overridesForUnknownMotorsAreAnError() throws Exception {
        HardwareMap map = robot();
        assertThrows(IllegalArgumentException.class, () -> parse("{\"motor_overrides\":{\"typo\":{\"static_friction_nm\":1}}}").apply(map));
    }

    @Test void rotorInertiaChangesHowFastAMotorSpinsUp() throws Exception {
        double[] speedAfter = new double[2];
        String[] inertias = {"0.0015", "0.05"};
        for (int i = 0; i < 2; i++) {
            HardwareMap map = robot();
            parse("{\"motor_defaults\":{\"rotor_inertia_kg_m2\":" + inertias[i] + "}}").apply(map);
            map.get(DcMotor.class, "left").setPower(1);
            for (int t = 1; t <= 5; t++) HardwareMapBuilder.tickMotors(map, 0.01, t * 10L);
            speedAfter[i] = map.get(SimDcMotorEx.class, "left").getOmegaRadS();
        }
        assertTrue(speedAfter[0] > 3 * speedAfter[1], "light rotor " + speedAfter[0] + " vs heavy " + speedAfter[1]);
    }

    @Test void frictionAndControllerGainsReachTheMotor() throws Exception {
        HardwareMap map = robot();
        parse("{\"motor_defaults\":{\"velocity_p_gain\":3,\"position_p_gain\":0.5}}").apply(map);
        var left = map.get(SimDcMotorEx.class, "left");
        assertEquals(3, left.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p);
        assertEquals(0.5, left.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p);

        HardwareMap loaded = robot();
        parse("{\"motor_defaults\":{\"viscous_friction_nm_s_per_rad\":0.05}}").apply(loaded);
        loaded.get(DcMotor.class, "left").setPower(1);
        for (int t = 1; t <= 100; t++) HardwareMapBuilder.tickMotors(loaded, 0.01, t * 10L);
        double loadedSpeed = loaded.get(SimDcMotorEx.class, "left").getOmegaRadS();
        assertTrue(loadedSpeed < 25 && loadedSpeed > 10, "viscous friction should hold the speed below no-load (32.7): " + loadedSpeed);
    }

    @Test void floatingMotorCoastsWhileBrakingMotorStops() throws Exception {
        double[] omega = new double[2];
        DcMotor.ZeroPowerBehavior[] behaviors = {DcMotor.ZeroPowerBehavior.FLOAT, DcMotor.ZeroPowerBehavior.BRAKE};
        for (int i = 0; i < 2; i++) {
            HardwareMap map = robot();
            var m = map.get(SimDcMotorEx.class, "left");
            m.setZeroPowerBehavior(behaviors[i]);
            m.setPower(1);
            long t = 0;
            for (int k = 0; k < 20; k++) HardwareMapBuilder.tickMotors(map, 0.01, t += 10);
            m.setPower(0);
            for (int k = 0; k < 10; k++) HardwareMapBuilder.tickMotors(map, 0.01, t += 10);
            omega[i] = m.getOmegaRadS();
        }
        assertTrue(omega[0] > 25, "float coasts: " + omega[0]);
        assertTrue(omega[1] < 1, "brake stops: " + omega[1]);
    }
}
