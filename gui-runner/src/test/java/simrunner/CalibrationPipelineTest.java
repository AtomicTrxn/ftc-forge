package simrunner;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import physics.BatteryModel;
import physics.MecanumKinematics;
import physics.MotorModel;
import physics.MotorSpec;
import physics.calibration.TelemetryLogRow;
import physics.calibration.TelemetryLogWriter;
import simcore.HardwareMapBuilder;
import simcore.SimDcMotorEx;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class CalibrationPipelineTest {
    @TempDir Path team;
    private static final String[] NAMES = {"left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive"};
    private static final MotorSpec SPEC = new MotorSpec("test", 19.2, 2.383, 9.2, 32.67, 12, 537.7);

    @Test void cliRecoversSyntheticProfileAndAppliesIt() throws Exception {
        Files.writeString(team.resolve("sim.config"), "{\"presetMotors\":\"motors.json\"}");
        StringBuilder preset = new StringBuilder("{\"name\":\"test\",\"motors\":{");
        for (int i = 0; i < 4; i++) {
            if (i > 0) preset.append(',');
            preset.append('"').append(NAMES[i]).append('"').append(":{\"sku\":\"test\",\"ratio\":19.2,")
                .append("\"tauStallNm\":2.383,\"iStallAmps\":9.2,\"omegaNoLoadRadS\":32.67,")
                .append("\"vNominal\":12,\"encoderCountsPerRev\":537.7}");
        }
        preset.append("}}");
        Files.writeString(team.resolve("motors.json"), preset.toString());
        writeMotorLog();
        writeDriveLog();
        CalibrationCli.main(new String[]{team.toString(), "motor.csv", "fit.json", ".0015",
            ".048", ".30", ".35", "drive.csv"});
        CalibrationProfile fit = CalibrationProfile.load(team.resolve("fit.json"));
        assertEquals(12.6, fit.vInternal, .06);
        assertEquals(.22, fit.rBattery, .015);
        assertEquals(4, fit.motors.size());
        assertEquals(.12, fit.motors.get(NAMES[0]).staticTorqueNm(), .03);
        assertEquals(.008, fit.motors.get(NAMES[0]).viscousBNmS(), .002);
        assertEquals(.10, fit.drive.responseTimeS(), .02);
        assertEquals(7.8, fit.drive.maxAccelMps2(), .75);
        assertEquals(.10, fit.drive.yawResponseTimeS(), .02);
        assertEquals(20, fit.drive.maxYawAccelRadps2(), 1.5);
        HardwareMap hardware = new HardwareMap();
        for (String name : NAMES) hardware.register(name, new SimDcMotorEx(name, SPEC));
        fit.applyHardware(hardware);
        assertEquals(fit.vInternal, HardwareMapBuilder.getBatteryModel().vInternal);
        assertEquals(fit.rBattery, HardwareMapBuilder.getBatteryModel().rBattery);
    }

    private void writeMotorLog() throws Exception {
        double[] omega = new double[4], ticks = new double[4];
        MotorModel[] models = new MotorModel[4];
        for (int i = 0; i < 4; i++) {
            models[i] = new MotorModel();
            models[i].tauStaticNm = .12; models[i].viscousBNms = .008;
        }
        BatteryModel battery = new BatteryModel(12.6, .22);
        try (TelemetryLogWriter writer = new TelemetryLogWriter(team.resolve("motor.csv"), List.of(NAMES))) {
            for (int step = 0; step < 150; step++) {
                double power = step % 30 < 10 ? .5 : step % 30 < 20 ? .7 : .9;
                List<BatteryModel.MotorState> states = new ArrayList<>();
                for (double w : omega) states.add(new BatteryModel.MotorState(power, SPEC, w));
                double voltage = battery.solveBatteryVoltage(states);
                List<TelemetryLogRow.MotorSample> samples = new ArrayList<>();
                for (int m = 0; m < 4; m++) {
                    double current = Math.abs(models[m].current(SPEC, omega[m], power * voltage));
                    double velocity = omega[m] / (2 * Math.PI) * SPEC.encoderCountsPerRev;
                    double torque = models[m].torque(SPEC, omega[m], power * voltage);
                    omega[m] += torque / .0015 * .02;
                    ticks[m] += omega[m] * .02 / (2 * Math.PI) * SPEC.encoderCountsPerRev;
                    samples.add(new TelemetryLogRow.MotorSample(power, Math.round(ticks[m]), velocity, current));
                }
                writer.logTick(step * 20L, 20, voltage, samples);
            }
        }
    }

    private void writeDriveLog() throws Exception {
        MecanumKinematics kinematics = new MecanumKinematics(.30, .35, 5);
        double vx = 0, vy = 0, yaw = 0;
        try (TelemetryLogWriter writer = new TelemetryLogWriter(team.resolve("drive.csv"), List.of(NAMES), true)) {
            for (int step = 0; step < 200; step++) {
                double commandVx = step % 32 < 8 ? 1.4 : step % 32 < 16 ? -1.4 : 0;
                double commandVy = step % 32 < 16 ? 0 : step % 32 < 24 ? 1.4 : -1.4;
                double commandYaw = step % 20 < 10 ? 3 : -3;
                double[] wheels = kinematics.inverse(commandVx, commandVy, commandYaw);
                List<TelemetryLogRow.MotorSample> motors = new ArrayList<>();
                for (double power : wheels) {
                    double speed = power * 5;
                    motors.add(new TelemetryLogRow.MotorSample(power, 0,
                        speed / .048 / (2 * Math.PI) * SPEC.encoderCountsPerRev, 3));
                }
                writer.logTick(step * 20L, 20, 12, motors, vx, vy, yaw);
                var target = kinematics.forwardFromWheelSpeeds(wheels[0] * 5, wheels[1] * 5,
                    wheels[2] * 5, wheels[3] * 5);
                double response = -Math.expm1(-.02 / .10);
                double dx = (target.vx - vx) * response, dy = (target.vy - vy) * response;
                double length = Math.hypot(dx, dy);
                if (length > 7.8 * .02) { dx *= 7.8 * .02 / length; dy *= 7.8 * .02 / length; }
                vx += dx; vy += dy;
                double delta = (target.omega - yaw) * response;
                yaw += Math.max(-20 * .02, Math.min(20 * .02, delta));
            }
        }
    }
}
