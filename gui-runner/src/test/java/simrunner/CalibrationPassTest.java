package simrunner;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import physics.calibration.TelemetryLogReader;
import physics.calibration.TelemetryLogRow;
import physics.calibration.TelemetryLogWriter;
import simcore.ElectricalConfig;
import simcore.HardwareMapBuilder;
import simcore.MiniJson;
import simcore.PresetRobotConfig;
import simcore.RobotConfigXml;
import simcore.SimDcMotorEx;
import simcore.SimIMU;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Closed-loop checks of the extra calibration stages: run the simulator's own models with known
 * parameters, record a log, fit it, and confirm the parameters come back. These validate the fitting
 * code; they say nothing about agreement with a physical robot.
 */
class CalibrationPassTest {
    private static final Path FIXTURE = Path.of("compat-fixtures");
    private static final List<String> MOTORS = List.of("left_front_drive", "right_front_drive");
    @TempDir Path team;

    private HardwareMap truthRobot(String config) throws Exception {
        HardwareMap map = HardwareMapBuilder.build(RobotConfigXml.parse(FIXTURE.resolve("robot_config.xml").toFile()),
            PresetRobotConfig.load(FIXTURE.resolve("preset_motors.json")));
        ElectricalConfig.parse(MiniJson.parseObject(config)).apply(map);
        return map;
    }

    /** Drives both motors with piecewise-constant random powers on a fixed 10 ms clock and records a log. */
    private Path recordMotorLog(HardwareMap map, int seconds, String name, boolean stepsOnly) throws Exception {
        Path csv = team.resolve(name);
        Random random = new Random(11);
        double power = 0;
        try (TelemetryLogWriter writer = new TelemetryLogWriter(csv, MOTORS)) {
            long t = 0;
            for (int tick = 0; tick < seconds * 100; tick++) {
                if (tick % (stepsOnly ? 50 : 100) == 0) power = Math.round((random.nextDouble() * 2 - 1) * 10) / 10.0;
                for (String m : MOTORS) map.get(DcMotor.class, m).setPower(power);
                t += 10;
                HardwareMapBuilder.tickMotors(map, 0.01, t);
                var samples = MOTORS.stream().map(m -> {
                    var motor = map.get(DcMotorEx.class, m);
                    return new TelemetryLogRow.MotorSample(motor.getPower(), motor.getCurrentPosition(), motor.getVelocity(),
                        motor.getCurrent(CurrentUnit.AMPS));
                }).toList();
                writer.logTick(t, 10, map.get(com.qualcomm.hardware.lynx.LynxModule.class, "Control Hub").getVoltage(), samples);
            }
        }
        return csv;
    }

    @Test void depletionStageRecoversEmptyVoltageAndResistanceRise() throws Exception {
        HardwareMap map = truthRobot("{\"battery\":{\"internal_resistance_ohm\":0.2,\"capacity_mah\":500,\"empty_voltage_v\":10.5,"
            + "\"empty_resistance_ohm\":0.5},\"motor_defaults\":{\"viscous_friction_nm_s_per_rad\":0.05},\"encoder_latency_ms\":0}");
        var rows = TelemetryLogReader.read(recordMotorLog(map, 150, "depletion.csv", false));
        CalibrationFitter fitter = new CalibrationFitter();
        fitter.fitDepletion(rows, 500);
        @SuppressWarnings("unchecked") Map<String, Object> battery = (Map<String, Object>) fitter.electrical.get("battery");
        assertNotNull(battery, fitter.notes.toString());
        assertEquals(12.6, ((Number) battery.get("internal_voltage_v")).doubleValue(), 0.05);
        assertEquals(0.2, ((Number) battery.get("internal_resistance_ohm")).doubleValue(), 0.03);
        assertEquals(10.5, ((Number) battery.get("empty_voltage_v")).doubleValue(), 0.3);
        assertEquals(0.5, ((Number) battery.get("empty_resistance_ohm")).doubleValue(), 0.12);
        fitter.validate();
    }

    @Test void depletionStageRefusesAShortRecording() throws Exception {
        HardwareMap map = truthRobot("{\"motor_defaults\":{\"viscous_friction_nm_s_per_rad\":0.05},\"encoder_latency_ms\":0}");
        var rows = TelemetryLogReader.read(recordMotorLog(map, 10, "short.csv", false));
        CalibrationFitter fitter = new CalibrationFitter();
        fitter.fitDepletion(rows, 3000);
        assertTrue(fitter.electrical.isEmpty());
        assertTrue(fitter.notes.get(0).contains("Battery depletion not fitted"), fitter.notes.toString());
    }

    @Test void inertiaAndFrictionAreRecoveredTogetherFromSpinUpTransients() throws Exception {
        HardwareMap map = truthRobot("{\"encoder_latency_ms\":0,\"motor_defaults\":{\"rotor_inertia_kg_m2\":0.006,"
            + "\"static_friction_nm\":0.08,\"viscous_friction_nm_s_per_rad\":0.01}}");
        var rows = TelemetryLogReader.read(recordMotorLog(map, 60, "inertia.csv", true));
        CalibrationFitter fitter = new CalibrationFitter();
        double[] friction = fitter.fitInertia(rows, "left_front_drive", PresetRobotConfig.load(FIXTURE.resolve("preset_motors.json")).motors.get("left_front_drive"));
        assertNotNull(friction, fitter.notes.toString());
        @SuppressWarnings("unchecked") Map<String, Object> entry = (Map<String, Object>) ((Map<String, Object>) fitter.electrical.get("motor_overrides")).get("left_front_drive");
        assertEquals(0.006, ((Number) entry.get("rotor_inertia_kg_m2")).doubleValue(), 0.0006);
        assertEquals(0.08, friction[0], 0.03);
        assertEquals(0.01, friction[1], 0.003);
    }

    @Test void inertiaIsRefusedWhenTheMotorNeverAccelerates() throws Exception {
        HardwareMap map = truthRobot("{\"encoder_latency_ms\":0}");
        Path csv = team.resolve("flat.csv");
        try (TelemetryLogWriter writer = new TelemetryLogWriter(csv, MOTORS)) {
            for (int i = 1; i <= 200; i++) {
                var sample = new TelemetryLogRow.MotorSample(0.5, i * 5L, 500, 1.0);
                writer.logTick(i * 10L, 10, 12.6, List.of(sample, sample));
            }
        }
        CalibrationFitter fitter = new CalibrationFitter();
        assertNull(fitter.fitInertia(TelemetryLogReader.read(csv), "left_front_drive",
            PresetRobotConfig.load(FIXTURE.resolve("preset_motors.json")).motors.get("left_front_drive")));
        assertTrue(fitter.notes.get(0).contains("not identifiable"), fitter.notes.toString());
    }

    @Test void velocityWindowIsRecovered() throws Exception {
        HardwareMap map = truthRobot("{\"encoder_latency_ms\":0,\"bus\":{\"velocity_window_ms\":60},"
            + "\"motor_defaults\":{\"viscous_friction_nm_s_per_rad\":0.05,\"rotor_inertia_kg_m2\":0.01}}");
        var rows = TelemetryLogReader.read(recordMotorLog(map, 60, "velocity.csv", true));
        CalibrationFitter fitter = new CalibrationFitter();
        fitter.fitVelocityWindow(rows, MOTORS);
        @SuppressWarnings("unchecked") Map<String, Object> bus = (Map<String, Object>) fitter.electrical.get("bus");
        assertNotNull(bus, fitter.notes.toString());
        assertEquals(60, ((Number) bus.get("velocity_window_ms")).doubleValue(), 10);
    }

    /** Reference yaw (rad) used by the sensor recording; non-monotone so scale and drift are separable. */
    private static double refYaw(double s) { return 1.5 * Math.sin(2 * Math.PI * s / 20); }
    private static double refYawRate(double s) { return 1.5 * 2 * Math.PI / 20 * Math.cos(2 * Math.PI * s / 20); }

    private Path recordSensorLog() throws Exception {
        HardwareMap map = truthRobot("{\"encoder_latency_ms\":0}");
        SimIMU imu = map.get(SimIMU.class, "imu");
        imu.setLatencyMs(40);
        imu.configureErrors(new simcore.ImuParams(0.03, 0.002, 0.004, 0.003, 0.002, 5, 0), "imu");
        GoBildaPinpointDriver pinpoint = map.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.configureOdometry(new simcore.OdometryParams(30, 0.02, -0.03, 0.001, 0.002, 0.003, 9, 0), "pinpoint");
        Path csv = team.resolve("sensors.csv");
        try (TelemetryLogWriter writer = new TelemetryLogWriter(csv, MOTORS, false, true)) {
            for (int tick = 0; tick <= 6000; tick++) {
                long t = tick * 10L;
                double s = t / 1000.0;
                boolean still = s < 5 || s > 55;                         // stationary stretches for gyro bias/noise
                double move = still ? (s < 5 ? 0 : 55 - 5) : s - 5;
                double yaw = still ? refYaw(s < 5 ? 0 : 50) : refYaw(move), rate = still ? 0 : refYawRate(move);
                double x = still ? 0 : Math.sin(2 * Math.PI * move / 15), y = still ? 0 : 0.5 * Math.sin(2 * Math.PI * move / 9);
                double heading = still ? 0 : 1.2 * Math.sin(2 * Math.PI * move / 12);
                imu.update(yaw, rate, t);
                pinpoint.onChassisPose(t, x, y, heading, 0, 0, 0);
                pinpoint.update();
                double imuYaw = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                double imuRate = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
                var motor = new TelemetryLogRow.MotorSample(0, 0, 0, 0);
                writer.logTick(t, 10, 12.6, List.of(motor, motor), null, null, null,
                    new TelemetryLogRow.SensorSample(imuYaw, yaw, imuRate, rate,
                        pinpoint.getPosX(DistanceUnit.METER), pinpoint.getPosY(DistanceUnit.METER), pinpoint.getHeading(AngleUnit.RADIANS),
                        x, y, heading));
            }
        }
        return csv;
    }

    @Test void imuAndOdometryErrorsAreRecoveredFromAReferenceRecording() throws Exception {
        var rows = TelemetryLogReader.read(recordSensorLog());
        CalibrationFitter fitter = new CalibrationFitter();
        fitter.fitSensors(rows);
        @SuppressWarnings("unchecked") Map<String, Object> imu = (Map<String, Object>) fitter.electrical.get("imu");
        @SuppressWarnings("unchecked") Map<String, Object> odo = (Map<String, Object>) fitter.electrical.get("odometry");
        assertNotNull(imu, fitter.notes.toString());
        assertNotNull(odo, fitter.notes.toString());
        assertEquals(40, ((Number) fitter.electrical.get("imu_latency_ms")).doubleValue(), 3);
        assertEquals(0.03, ((Number) imu.get("yaw_scale_error")).doubleValue(), 0.005);
        assertEquals(0.002, ((Number) imu.get("yaw_drift_rad_s")).doubleValue(), 0.0006);
        assertEquals(0.004, ((Number) imu.get("gyro_bias_rad_s")).doubleValue(), 0.001);
        assertEquals(0.003, ((Number) imu.get("gyro_noise_rad_s")).doubleValue(), 0.001);
        assertEquals(0.002, ((Number) imu.get("angle_noise_rad")).doubleValue(), 0.001);

        assertEquals(30, ((Number) odo.get("latency_ms")).doubleValue(), 3);
        assertEquals(0.02, ((Number) odo.get("linear_scale_error")).doubleValue(), 0.004);
        assertEquals(-0.03, ((Number) odo.get("heading_scale_error")).doubleValue(), 0.006);
        assertEquals(0.001, ((Number) odo.get("heading_drift_rad_s")).doubleValue(), 0.0004);
        assertEquals(0.002, ((Number) odo.get("position_noise_m")).doubleValue(), 0.001);
        fitter.validate();
    }

    @Test void sensorStagesSkipWithNotesWhenTheReferenceDoesNotMove() throws Exception {
        Path csv = team.resolve("still.csv");
        try (TelemetryLogWriter writer = new TelemetryLogWriter(csv, MOTORS, false, true)) {
            for (int i = 1; i <= 200; i++) {
                var motor = new TelemetryLogRow.MotorSample(0, 0, 0, 0);
                writer.logTick(i * 10L, 10, 12.6, List.of(motor, motor), null, null, null,
                    new TelemetryLogRow.SensorSample(0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0));
            }
        }
        CalibrationFitter fitter = new CalibrationFitter();
        fitter.fitSensors(TelemetryLogReader.read(csv));
        assertFalse(fitter.electrical.containsKey("odometry"));
        assertFalse(fitter.electrical.containsKey("imu_latency_ms"));
        assertTrue(fitter.notes.stream().anyMatch(n -> n.contains("IMU yaw not fitted")), fitter.notes.toString());
        assertTrue(fitter.notes.stream().anyMatch(n -> n.contains("Odometry position not fitted")), fitter.notes.toString());
    }

    @Test void sensorColumnsMustBePairedWithTheirReference() throws Exception {
        Path csv = team.resolve("bad.csv");
        Files.writeString(csv, "t_ms,loop_iter,loop_time_ms,battery_voltage_v,motor_a_power,motor_a_ticks,motor_a_vel_tps,motor_a_current_a,"
            + String.join(",", TelemetryLogRow.SensorSample.COLUMNS) + "\n"
            + "10,0,10,12,0,0,0,0,0.1,,,,,,,,,\n".repeat(1));
        assertThrows(IllegalArgumentException.class, () -> TelemetryLogReader.read(csv));
    }

    @Test void endToEndProfileOverlaysSimConfigAndAppliesToAFreshRobot() throws Exception {
        HardwareMap truth = truthRobot("{\"battery\":{\"internal_resistance_ohm\":0.2,\"capacity_mah\":500,\"empty_voltage_v\":10.5,"
            + "\"empty_resistance_ohm\":0.5},\"motor_defaults\":{\"viscous_friction_nm_s_per_rad\":0.05},\"encoder_latency_ms\":0}");
        recordMotorLog(truth, 150, "motor.csv", false);
        recordSensorLog();
        Files.copy(FIXTURE.resolve("preset_motors.json"), team.resolve("preset_motors.json"));
        Files.copy(FIXTURE.resolve("robot_config.xml"), team.resolve("robot_config.xml"));
        Files.writeString(team.resolve("sim.config"), "{\"sourceRoot\":\"TeamCode/src/main/java\",\"robotConfig\":\"robot_config.xml\","
            + "\"presetMotors\":\"preset_motors.json\",\"imu_latency_ms\":8,\"calibration\":\"calibration.json\","
            + "\"odometry\":{\"seed\":77,\"latency_ms\":5}}");

        CalibrationCli.main(new String[]{team.toString(), "motor.csv", "calibration.json", "0.0015", "0.048", "0.3", "0.35",
            "capacity_mah=500", "sensor_log=sensors.csv"});

        @SuppressWarnings("unchecked") Map<String, Object> profile = MiniJson.parseObject(Files.readString(team.resolve("calibration.json")));
        @SuppressWarnings("unchecked") Map<String, Object> electrical = (Map<String, Object>) profile.get("electrical");
        assertNotNull(electrical);
        assertTrue(profile.containsKey("fit_notes"));

        SimConfig config = SimConfig.load(team);
        HardwareMap fresh = HardwareMapBuilder.build(RobotConfigXml.parse(team.resolve("robot_config.xml").toFile()),
            PresetRobotConfig.load(team.resolve("preset_motors.json")));
        config.applyElectrical(fresh, team);
        assertEquals(40, fresh.get(SimIMU.class, "imu").getLatencyMs(), 3, "fitted IMU latency replaces sim.config's 8 ms");
        assertEquals(0.5, HardwareMapBuilder.getBatteryPack().emptyResistanceOhm, 0.12);
        assertEquals(0.5, HardwareMapBuilder.getBatteryPack().capacityAh, 1e-9);
        // sim.config's own odometry seed survives the overlay (the profile never fits a seed).
        assertNotNull(fresh.get(GoBildaPinpointDriver.class, "pinpoint"));
    }

    @Test void profilesWithInvalidElectricalBlocksAreRejectedAtLoad() throws Exception {
        Path profile = team.resolve("bad.json");
        Files.writeString(profile, "{\"battery\":{\"v_internal\":12.6,\"r_battery\":0.15},\"electrical\":{\"imu\":{\"nonsense\":1}}}");
        assertThrows(IllegalArgumentException.class, () -> CalibrationProfile.load(profile));
        Files.writeString(profile, "{\"battery\":{\"v_internal\":12.6,\"r_battery\":0.15},\"electrical\":5}");
        assertThrows(IllegalArgumentException.class, () -> CalibrationProfile.load(profile));
    }
}
