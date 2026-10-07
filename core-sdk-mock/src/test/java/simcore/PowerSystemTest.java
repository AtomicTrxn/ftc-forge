package simcore;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/** Battery state of charge, brownout, servo/hub loads and hub read latency, on a fixed (deterministic) clock. */
class PowerSystemTest {
    @TempDir Path dir;

    private HardwareMap robot(String configJson) throws Exception {
        Path xml = dir.resolve("robot.xml");
        Files.writeString(xml, "<Robot><LynxUsbDevice name=\"u\"><LynxModule name=\"Control Hub\">"
            + "<goBILDA5202SeriesMotor name=\"a\" port=\"0\"/><goBILDA5202SeriesMotor name=\"b\" port=\"1\"/>"
            + "<Servo name=\"claw\" port=\"0\"/><CRServo name=\"roller\" port=\"1\"/></LynxModule></LynxUsbDevice></Robot>");
        Path preset = dir.resolve("preset.json");
        Files.writeString(preset, "{\"name\":\"p\",\"motors\":{"
            + "\"a\":{\"sku\":\"t\",\"ratio\":19.2,\"tauStallNm\":2.4,\"iStallAmps\":9.2,\"omegaNoLoadRadS\":32.7,\"vNominal\":12,\"encoderCountsPerRev\":537.7},"
            + "\"b\":{\"sku\":\"t\",\"ratio\":19.2,\"tauStallNm\":2.4,\"iStallAmps\":9.2,\"omegaNoLoadRadS\":32.7,\"vNominal\":12,\"encoderCountsPerRev\":537.7}}}");
        HardwareMap map = HardwareMapBuilder.build(RobotConfigXml.parse(xml.toFile()), PresetRobotConfig.load(preset));
        ElectricalConfig.parse(parseJson(configJson)).apply(map);
        return map;
    }

    private static Map<String, Object> parseJson(String json) { return MiniJson.parseObject(json); }

    private static long clock;
    private static void tick(HardwareMap map, int count) {
        for (int i = 0; i < count; i++) { clock += 10; HardwareMapBuilder.tickMotors(map, 0.01, clock); }
    }
    private static void reset() { clock = 0; }
    private static double volts(HardwareMap map) { return map.get(LynxModule.class, "Control Hub").getVoltage(); }

    @Test void chargeDrainsAndOpenCircuitVoltageFallsAsConfigured() throws Exception {
        reset();
        HardwareMap map = robot("{\"battery\":{\"capacity_mah\":50,\"empty_voltage_v\":10.0,\"internal_resistance_ohm\":0},"
            + "\"motor_defaults\":{\"viscous_friction_nm_s_per_rad\":0.05}}");   // friction gives the motors a steady load
        var motors = map.getAll(DcMotor.class);
        motors.forEach(m -> m.setPower(1));
        tick(map, 5);
        double early = volts(map);
        tick(map, 2000);                                  // 20 s of driving a 50 mAh pack
        assertTrue(HardwareMapBuilder.getBatteryPack().chargeFraction() < 0.9, "charge " + HardwareMapBuilder.getBatteryPack().chargeFraction());
        assertTrue(volts(map) < early - 0.2, early + " -> " + volts(map));
    }

    @Test void stateOfChargeIsOffByDefault() throws Exception {
        reset();
        HardwareMap map = robot("{}");
        map.getAll(DcMotor.class).forEach(m -> m.setPower(1));
        tick(map, 3000);
        assertEquals(1.0, HardwareMapBuilder.getBatteryPack().chargeFraction());
    }

    @Test void brownoutCutsMotorsHoldsThenRecovers() throws Exception {
        reset();
        // At stall two motors sag the pack to about 9.7 V, below the 10 V brownout threshold.
        HardwareMap map = robot("{\"battery\":{\"brownout_voltage_v\":10,\"brownout_hold_s\":1,\"internal_resistance_ohm\":0.4}}");
        var a = map.get(SimDcMotorEx.class, "a");
        a.setPower(1);
        map.get(SimDcMotorEx.class, "b").setPower(1);
        tick(map, 2);
        assertTrue(HardwareMapBuilder.getBatteryPack().isBrownedOut());
        tick(map, 1);
        assertTrue(a.isPowerCut());
        assertEquals(0, a.commandedPower());
        assertEquals(0, a.getCurrent(CurrentUnit.AMPS), 1e-9, "driver off draws no current");
        assertEquals(12.6, volts(map), 1e-6, "no load once the motors are cut");
        int recoveredAtTick = -1;
        for (int i = 0; i < 200 && recoveredAtTick < 0; i++) {
            tick(map, 1);
            if (!HardwareMapBuilder.getBatteryPack().isBrownedOut()) recoveredAtTick = i;
        }
        // Browned out at tick ~2 with a 1 s hold: recovery cannot come before about tick 100.
        assertTrue(recoveredAtTick >= 95, "recovered too early at tick " + recoveredAtTick);
        tick(map, 3);                                      // the commands are still full power, so it sags again
        assertTrue(HardwareMapBuilder.getBatteryPack().isBrownedOut());
    }

    @Test void brownoutIsDisabledByDefault() throws Exception {
        reset();
        HardwareMap map = robot("{\"battery\":{\"internal_resistance_ohm\":0.5}}");
        map.getAll(DcMotor.class).forEach(m -> m.setPower(1));
        tick(map, 5);
        assertFalse(HardwareMapBuilder.getBatteryPack().isBrownedOut());
    }

    @Test void hubBaselineAndServoLoadsSagTheBattery() throws Exception {
        reset();
        HardwareMap map = robot("{\"loads\":{\"hub_baseline_a\":1.0,\"servo_hold_a\":0.5,\"servo_active_a\":2.0,\"servo_active_duration_ms\":100}}");
        tick(map, 2);
        assertEquals(12.6 - 0.15 * 1.0, volts(map), 1e-9, "baseline only: servo never commanded");
        Servo claw = map.get(Servo.class, "claw");
        claw.setPosition(0.8);
        tick(map, 1);
        assertEquals(12.6 - 0.15 * (1.0 + 2.0), volts(map), 1e-9, "servo moving");
        tick(map, 20);                                     // 200 ms later the move is over
        assertEquals(12.6 - 0.15 * (1.0 + 0.5), volts(map), 1e-9, "servo holding");
        map.get(CRServo.class, "roller").setPower(-0.5);
        tick(map, 1);
        assertEquals(12.6 - 0.15 * (1.0 + 0.5 + 1.0), volts(map), 1e-9, "continuous servo at half power draws half the active current");
    }

    @Test void hubVoltageReadsAreDelayedByTheConfiguredLatency() throws Exception {
        reset();
        HardwareMap map = robot("{\"bus\":{\"voltage_latency_ms\":50}}");
        tick(map, 20);
        assertEquals(12.6, volts(map), 1e-9);
        map.getAll(DcMotor.class).forEach(m -> m.setPower(1));
        tick(map, 1);                                      // the sag is published now but read 50 ms late
        assertEquals(12.6, volts(map), 1e-9);
        tick(map, 6);
        assertTrue(volts(map) < 12.0, "sag visible after the latency: " + volts(map));
    }

    @Test void motorCurrentReadsAreDelayedByTheConfiguredLatency() throws Exception {
        reset();
        HardwareMap map = robot("{\"bus\":{\"current_latency_ms\":50},\"motor_defaults\":{\"viscous_friction_nm_s_per_rad\":0.05}}");
        var a = map.get(SimDcMotorEx.class, "a");
        tick(map, 5);
        a.setPower(1);
        tick(map, 2);
        assertEquals(0, a.getCurrent(CurrentUnit.AMPS), 1e-9, "still the old reading");
        tick(map, 8);
        assertTrue(a.getCurrent(CurrentUnit.AMPS) > 0.5);
    }

    @Test void defaultsLeaveVoltageAndCurrentReadsImmediate() throws Exception {
        reset();
        HardwareMap map = robot("{\"motor_defaults\":{\"viscous_friction_nm_s_per_rad\":0.05}}");
        var a = map.get(SimDcMotorEx.class, "a");
        a.setPower(1);
        tick(map, 2);
        assertTrue(a.getCurrent(CurrentUnit.AMPS) > 0.5);
        assertTrue(volts(map) < 12.6);
    }
}
