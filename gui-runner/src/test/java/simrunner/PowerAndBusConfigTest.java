package simrunner;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import physics.BatteryModel;
import simcore.HardwareMapBuilder;
import simcore.PresetRobotConfig;
import simcore.RobotConfigXml;
import simcore.SimDcMotorEx;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Battery sag and encoder/bus latency behave as configured, end to end through sim.config. */
class PowerAndBusConfigTest {
    private static final Path PROJECT = Path.of("compat-fixtures");
    private static final List<String> DRIVE = List.of("left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive");
    @TempDir Path team;

    private HardwareMap build(String simConfigJson) throws Exception {
        Files.writeString(team.resolve("sim.config"), simConfigJson);
        SimConfig config = SimConfig.load(team);
        HardwareMap map = HardwareMapBuilder.build(RobotConfigXml.parse(PROJECT.resolve("robot_config.xml").toFile()),
            PresetRobotConfig.load(PROJECT.resolve("preset_motors.json")));
        config.applyElectrical(map);
        return map;
    }

    private static void drive(HardwareMap map, double power) {
        for (String name : DRIVE) map.get(DcMotor.class, name).setPower(power);
    }

    private static double hubVoltage(HardwareMap map) { return map.voltageSensor.iterator().next().getVoltage(); }

    private static void tickFor(HardwareMap map, int ticks) throws InterruptedException {
        for (int i = 0; i < ticks; i++) { HardwareMapBuilder.tickMotors(map); Thread.sleep(10); }
    }

    private static double expectedVoltage(HardwareMap map) {
        var states = map.getAll(SimDcMotorEx.class).stream().filter(m -> DRIVE.contains(m.getDeviceName()))
            .map(m -> new BatteryModel.MotorState(m.signedCommandedPower(), m.getSpec(), m.getOmegaRadS())).toList();
        return HardwareMapBuilder.getBatteryModel().solveBatteryVoltage(states);
    }

    @Test void idleRobotShowsOpenCircuitVoltage() throws Exception {
        HardwareMap map = build("{\"battery\":{\"internal_voltage_v\":12.9}}");
        tickFor(map, 3);
        assertEquals(12.9, hubVoltage(map), 1e-9);
    }

    @Test void hubVoltageMatchesTheClosedFormSolveAndSagsHardAtStall() throws Exception {
        HardwareMap map = build("{}");
        tickFor(map, 2);
        drive(map, 1.0);
        double expected = expectedVoltage(map);          // state before the tick that publishes the voltage
        Thread.sleep(10);
        HardwareMapBuilder.tickMotors(map);
        assertEquals(expected, hubVoltage(map), 1e-9);
        // Four stalled 9.2 A motors through 0.15 ohm: V = 12.6 / (1 + 0.15 * 4 * 9.2 / 12), about 8.6 V.
        assertEquals(12.6 / (1 + 0.15 * 4 * 9.2 / 12), expected, 0.05);
    }

    /** The first tick only starts the clock, so run one before relying on published voltages. */
    private static void startClock(HardwareMap map) { HardwareMapBuilder.tickMotors(map); }

    private static double minVoltageWhileSpinningUp(HardwareMap map) throws InterruptedException {
        startClock(map);
        drive(map, 1.0);
        double min = hubVoltage(map);
        for (int i = 0; i < 20; i++) {
            Thread.sleep(10);
            HardwareMapBuilder.tickMotors(map);
            min = Math.min(min, hubVoltage(map));
        }
        return min;
    }

    @Test void voltageRecoversWhenTheLoadIsRemoved() throws Exception {
        HardwareMap map = build("{}");
        assertTrue(minVoltageWhileSpinningUp(map) < 10.5);
        drive(map, 0);
        tickFor(map, 2);
        assertEquals(12.6, hubVoltage(map), 1e-9);
    }

    @Test void higherInternalResistanceMeansMoreSagDuringSpinUp() throws Exception {
        double stiff = minVoltageWhileSpinningUp(build("{\"battery\":{\"internal_resistance_ohm\":0.0}}"));
        double normal = minVoltageWhileSpinningUp(build("{}"));
        double weak = minVoltageWhileSpinningUp(build("{\"battery\":{\"internal_resistance_ohm\":0.4}}"));
        assertEquals(12.6, stiff, 1e-9);
        assertTrue(normal < stiff && weak < normal, stiff + " > " + normal + " > " + weak + " expected");
    }

    @Test void calibrationBatteryValuesOverrideSimConfig() throws Exception {
        HardwareMap map = build("{\"battery\":{\"internal_resistance_ohm\":0.4}}");
        startClock(map);
        HardwareMapBuilder.getBatteryModel().rBattery = 0.22;   // what a loaded calibration profile does afterwards
        drive(map, 1.0);
        double expected = expectedVoltage(map);
        Thread.sleep(10);
        HardwareMapBuilder.tickMotors(map);
        assertEquals(expected, hubVoltage(map), 1e-9);
        assertEquals(12.6 / (1 + 0.22 * 4 * 9.2 / 12), expected, 0.05);
    }

    @Test void encoderLatencyFromSimConfigReachesEveryMotor() throws Exception {
        HardwareMap map = build("{\"encoder_latency_ms\":25}");
        assertTrue(map.getAll(SimDcMotorEx.class).stream().allMatch(m -> m.getEncoderLatencyMs() == 25));
        assertTrue(build("{}").getAll(SimDcMotorEx.class).stream().allMatch(m -> m.getEncoderLatencyMs() == 8));
    }

    @Test void invalidElectricalSettingsAreRejected() throws Exception {
        for (String bad : List.of("{\"encoder_latency_ms\":-1}", "{\"encoder_latency_ms\":201}",
                "{\"battery\":{\"internal_resistance_ohm\":-0.1}}", "{\"battery\":{\"internal_resistance_ohm\":5}}",
                "{\"battery\":{\"internal_voltage_v\":3}}", "{\"battery\":{\"resistance\":0.1}}", "{\"battery\":12}")) {
            Files.writeString(team.resolve("sim.config"), bad);
            assertThrows(IllegalArgumentException.class, () -> SimConfig.load(team), bad);
        }
    }
}
