package simcore;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import physics.MotorSpec;

import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

class SensorErrorModelTest {
    @TempDir Path dir;

    private static SimIMU imu(ImuParams params, String name) {
        SimIMU imu = new SimIMU(name);
        imu.setLatencyMs(0);
        imu.configureErrors(params, name);
        return imu;
    }

    private static void feed(SimIMU imu, double yawRad, double rateRadS, long timeMs) {
        imu.update(yawRad, rateRadS, timeMs);
    }

    @Test void idealImuIsUnchanged() {
        SimIMU imu = imu(ImuParams.IDEAL, "imu");
        feed(imu, 0, 0, 0);
        feed(imu, 1.0, 2.0, 1000);
        assertEquals(Math.toDegrees(1.0), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 1e-9);
        assertEquals(2.0, imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, 1e-12);
    }

    @Test void yawScaleErrorBiasAndDriftApplyAndResetClearsDrift() {
        SimIMU imu = imu(new ImuParams(0.1, Math.toRadians(1), 0.02, 0, 0, 0, 0), "imu");
        feed(imu, 0, 0, 0);
        feed(imu, Math.toRadians(90), 1.0, 10_000);
        // 90 deg * 1.1 + 1 deg/s * 10 s
        assertEquals(109, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 1e-9);
        assertEquals(1.0 * 1.1 + 0.02, imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, 1e-12);
        assertEquals(0.02, imu.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate, 1e-12);
        imu.resetYaw();
        feed(imu, Math.toRadians(90), 0, 20_000);
        assertEquals(10, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 1e-9, "only the 10 s since reset drifted");
    }

    @Test void imuNoiseIsRepeatableByNameAndHasTheRequestedSpread() {
        ImuParams noisy = new ImuParams(0, 0, 0, 0.01, 0.002, 5, 0);
        double a = reading(noisy, "imu", 0), b = reading(noisy, "imu", 0), c = reading(noisy, "other", 0);
        assertEquals(a, b, 0.0);
        assertNotEquals(a, c);
        double sumSq = 0;
        int n = 400;
        for (int i = 0; i < n; i++) { double r = reading(new ImuParams(0, 0, 0, 0.01, 0, i, 0), "imu", 0); sumSq += r * r; }
        assertEquals(0.01, Math.sqrt(sumSq / n), 0.002, "gyro noise std");
    }

    private static double reading(ImuParams p, String name, double rate) {
        SimIMU imu = imu(p, name);
        feed(imu, 0, rate, 0);
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    @Test void imuReadCostBlocksTheCallerButNotTheSamplePublisher() throws Exception {
        SimIMU imu = imu(new ImuParams(0, 0, 0, 0, 0, 0, 5), "imu");
        feed(imu, 0, 0, 0);
        long start = System.nanoTime();
        for (int i = 0; i < 6; i++) imu.getRobotYawPitchRollAngles();
        assertTrue((System.nanoTime() - start) / 1e6 >= 28);
        // A slow reader must not stall the physics thread that publishes samples.
        Thread reader = new Thread(() -> { for (int i = 0; i < 20; i++) imu.getRobotAngularVelocity(AngleUnit.RADIANS); });
        reader.start();
        long publishStart = System.nanoTime();
        for (int t = 1; t <= 50; t++) feed(imu, 0, 0, t);
        assertTrue((System.nanoTime() - publishStart) / 1e6 < 50, "publishing must not wait on reads");
        reader.join();
    }

    @Test void imuParametersAreValidated() {
        assertThrows(IllegalArgumentException.class, () -> new ImuParams(0.9, 0, 0, 0, 0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> new ImuParams(0, 0, 0, -1, 0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> new ImuParams(0, 0, 0, 0, 0, 0, 100));
        assertThrows(IllegalArgumentException.class, () -> new ImuParams(0, Double.NaN, 0, 0, 0, 0, 0));
    }

    @Test void odometryReadCostIsPaidOncePerTransaction() {
        OdometryParams slow = new OdometryParams(0, 0, 0, 0, 0, 0, 0, 5);
        GoBildaPinpointDriver pinpoint = new GoBildaPinpointDriver("pp");
        pinpoint.configureOdometry(slow, "pp");
        long start = System.nanoTime();
        for (int i = 0; i < 6; i++) pinpoint.update();
        assertTrue((System.nanoTime() - start) / 1e6 >= 28);

        SparkFunOTOS otos = new SparkFunOTOS("otos");
        otos.configureOdometry(slow, "otos");
        var pos = new SparkFunOTOS.Pose2D(); var vel = new SparkFunOTOS.Pose2D(); var acc = new SparkFunOTOS.Pose2D();
        start = System.nanoTime();
        for (int i = 0; i < 4; i++) otos.getPosVelAcc(pos, vel, acc);
        double ms = (System.nanoTime() - start) / 1e6;
        assertTrue(ms >= 18 && ms < 60, "one 5 ms transaction per block read, took " + ms);
    }

    private HardwareMap robot(String config) throws Exception {
        Path xml = dir.resolve("robot.xml");
        Files.writeString(xml, "<Robot><LynxUsbDevice name=\"u\"><LynxModule name=\"Hub\"><goBILDA5202SeriesMotor name=\"m\" port=\"0\"/></LynxModule></LynxUsbDevice></Robot>");
        Path preset = dir.resolve("preset.json");
        Files.writeString(preset, "{\"name\":\"p\",\"motors\":{\"m\":{\"sku\":\"t\",\"ratio\":19.2,\"tauStallNm\":2.4,\"iStallAmps\":9.2,\"omegaNoLoadRadS\":32.7,\"vNominal\":12,\"encoderCountsPerRev\":1000}}}");
        HardwareMap map = HardwareMapBuilder.build(RobotConfigXml.parse(xml.toFile()), PresetRobotConfig.load(preset));
        ElectricalConfig.parse(MiniJson.parseObject(config)).apply(map);
        return map;
    }

    /** Shaft turning at {@code radS}, ticked every 10 ms for 1 s; returns the reported velocity at the end. */
    private double velocity(String config, double radS) throws Exception {
        HardwareMap map = robot(config);
        SimDcMotorEx motor = map.get(SimDcMotorEx.class, "m");
        motor.useExternalShaft();
        double rad = 0;
        for (int t = 10; t <= 1000; t += 10) {
            motor.syncExternalShaft(rad, radS);
            motor.integrate(12, 0.01, t);
            rad += radS * 0.01;
        }
        return ((DcMotorEx) motor).getVelocity();
    }

    @Test void velocityWindowQuantizesAndSmoothsWhileZeroKeepsTheExactSpeed() throws Exception {
        double slow = 2 * Math.PI * 0.0033;                       // 3.3 ticks/s on a 1000-count encoder
        assertEquals(3.3, velocity("{}", slow), 1e-6);
        double windowed = velocity("{\"bus\":{\"velocity_window_ms\":100}}", slow);
        assertEquals(0, windowed % 10, 1e-9, "100 ms of whole counts moves in 10 ticks/s steps: " + windowed);

        double fast = 2 * Math.PI * 1.0;                          // 1000 ticks/s
        assertEquals(1000, velocity("{\"bus\":{\"velocity_window_ms\":100}}", fast), 10 + 1e-6);
    }

    @Test void resistanceRisesAsThePackDrains() throws Exception {
        HardwareMap map = robot("{\"battery\":{\"capacity_mah\":3000,\"initial_charge_fraction\":0.2,\"empty_voltage_v\":10.5,"
            + "\"empty_resistance_ohm\":0.6},\"loads\":{\"hub_baseline_a\":5}}");
        HardwareMapBuilder.tickMotors(map, 0.01, 10);
        // Voc = 10.5 + 2.1 * 0.2; R = 0.15 + 0.45 * 0.8; V = Voc - R * 5
        double expected = (10.5 + 2.1 * 0.2) - (0.15 + 0.45 * 0.8) * 5;
        assertEquals(expected, map.get(com.qualcomm.hardware.lynx.LynxModule.class, "Hub").getVoltage(), 0.01);

        HardwareMap fresh = robot("{\"battery\":{\"capacity_mah\":3000,\"empty_resistance_ohm\":0.6},\"loads\":{\"hub_baseline_a\":5}}");
        HardwareMapBuilder.tickMotors(fresh, 0.01, 10);
        assertEquals(12.6 - 0.15 * 5, fresh.get(com.qualcomm.hardware.lynx.LynxModule.class, "Hub").getVoltage(), 0.01);
    }
}
