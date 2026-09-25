package physics.calibration;

import org.junit.jupiter.api.Test;
import physics.BatteryModel;
import physics.MotorModel;
import physics.MotorSpec;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Validates the staged calibration fit per Phase 3's Definition of Done: "Calibration pass
 * runs its staged fit against at least one sample telemetry log with a sane result."
 *
 * No physical robot is available in this environment (this is a desktop simulator repo,
 * not a real FTC robot), so this generates a SYNTHETIC log from known ground-truth
 * parameters using the exact same forward model the simulator itself runs, then confirms
 * the calibrator recovers those parameters from the log alone -- a standard, legitimate way
 * to validate a system-identification algorithm's correctness (fit against data with known
 * answers), clearly distinct from validating against a real recorded robot log.
 */
class CalibratorTest {

    private static final double ROTATIONAL_INERTIA = 0.0015;
    private static final double GROUND_TRUTH_R_BATTERY = 0.22;
    private static final double GROUND_TRUTH_V_INTERNAL = 12.6;
    private static final double GROUND_TRUTH_TAU_STATIC = 0.12;
    private static final double GROUND_TRUTH_VISCOUS_B = 0.008;

    @Test
    void stagedFit_recoversGroundTruthParametersFromASyntheticLog() throws IOException {
        MotorSpec spec = new MotorSpec("goBILDA-5203-19.2", 19.2, 2.383, 9.2, 32.67, 12.0, 537.7);
        Path logPath = Files.createTempFile("calibration-test-", ".csv");

        generateSyntheticLog(logPath, spec);
        List<TelemetryLogRow> rows = TelemetryLogReader.read(logPath);
        assertTrue(rows.size() > 50, "expected a reasonably long synthetic log, got " + rows.size());

        Calibrator.BatteryFit batteryFit = Calibrator.fitBattery(rows);
        assertEquals(GROUND_TRUTH_R_BATTERY, batteryFit.rBattery, 0.01,
            "R_battery recovered from the synthetic log should match ground truth");
        assertEquals(GROUND_TRUTH_V_INTERNAL, batteryFit.vInternal, 0.05,
            "V_internal recovered from the synthetic log should match ground truth");

        Calibrator.FrictionFit frictionFit = Calibrator.fitMotorFriction(
            rows, "test_motor", spec, ROTATIONAL_INERTIA, batteryFit.vInternal, batteryFit.rBattery);
        assertEquals(GROUND_TRUTH_TAU_STATIC, frictionFit.tauStaticNm, 0.03,
            "tauStatic recovered from the synthetic log should match ground truth");
        assertEquals(GROUND_TRUTH_VISCOUS_B, frictionFit.viscousBNms, 0.002,
            "viscousB recovered from the synthetic log should match ground truth");

        Files.deleteIfExists(logPath);
    }

    /** Runs the exact same forward model the simulator uses (single motor), with known parameters, and logs it. */
    private void generateSyntheticLog(Path path, MotorSpec spec) throws IOException {
        MotorModel model = new MotorModel();
        model.tauStaticNm = GROUND_TRUTH_TAU_STATIC;
        model.viscousBNms = GROUND_TRUTH_VISCOUS_B;
        BatteryModel battery = new BatteryModel(GROUND_TRUTH_V_INTERNAL, GROUND_TRUTH_R_BATTERY);

        double omega = 0;
        double ticks = 0;
        double dtSeconds = 0.02;
        long tMs = 0;
        double commandedPower = 0.7;

        try (TelemetryLogWriter writer = new TelemetryLogWriter(path, List.of("test_motor"))) {
            for (int step = 0; step < 150; step++) {
                double vBattery = battery.solveBatteryVoltage(
                    List.of(new BatteryModel.MotorState(commandedPower, spec, omega)));
                double vActual = commandedPower * vBattery;

                // Current is drawn at the SAME (pre-integration) omega that determined vBattery
                // this tick -- computing it after updating omega would desynchronize the
                // logged (current, voltage) pair from the state that actually produced it.
                double current = model.current(spec, omega, vActual);
                double velTps = (omega / (2 * Math.PI)) * spec.encoderCountsPerRev;

                double tau = model.torque(spec, omega, vActual);
                double alpha = tau / ROTATIONAL_INERTIA;
                omega += alpha * dtSeconds;
                ticks += (omega * dtSeconds / (2 * Math.PI)) * spec.encoderCountsPerRev;

                writer.logTick(tMs, dtSeconds * 1000, vBattery,
                    List.of(new TelemetryLogRow.MotorSample(commandedPower, Math.round(ticks), velTps, current)));

                tMs += Math.round(dtSeconds * 1000);
            }
        }
    }
}
