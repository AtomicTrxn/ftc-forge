package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/** Copy this single source file into a real FTC TeamCode project to record calibration data. */
public final class CalibrationRecorder implements AutoCloseable {
    private final BufferedWriter writer;
    private final HardwareMap hardwareMap;
    private final List<String> names;
    private final List<DcMotorEx> motors = new ArrayList<>();
    private final long startedNanos = System.nanoTime();
    private long previousNanos = startedNanos;
    private int iteration;

    public CalibrationRecorder(File csv, HardwareMap hardwareMap, List<String> motorNames) throws IOException {
        if (motorNames.isEmpty()) throw new IllegalArgumentException("At least one motor is required");
        for (String name : motorNames) {
            if (!name.matches("[A-Za-z0-9_]+")) throw new IllegalArgumentException("Invalid motor name for CSV: " + name);
            motors.add(hardwareMap.get(DcMotorEx.class, name));
        }
        File parent = csv.getParentFile();
        if (parent != null && !parent.isDirectory() && !parent.mkdirs())
            throw new IOException("Cannot create calibration log directory " + parent);
        this.hardwareMap = hardwareMap;
        this.names = new ArrayList<String>(motorNames);
        this.writer = new BufferedWriter(new FileWriter(csv));
        StringBuilder header = new StringBuilder("t_ms,loop_iter,loop_time_ms,battery_voltage_v");
        for (String name : names) header.append(",motor_").append(name).append("_power")
            .append(",motor_").append(name).append("_ticks")
            .append(",motor_").append(name).append("_vel_tps")
            .append(",motor_").append(name).append("_current_a");
        header.append(",vx_mps,vy_mps,omega_rad_s");
        writer.write(header.toString()); writer.newLine();
    }

    /** Call once per OpMode loop. Motor data alone fits battery and motor parameters. */
    public void record() throws IOException { record(null, null, null); }

    /** Supply robot-frame +forward/+left velocity and +CCW yaw rate from localization if available. */
    public void record(Double vxMps, Double vyMps, Double omegaRadS) throws IOException {
        if ((vxMps == null) != (vyMps == null) || (vxMps == null) != (omegaRadS == null))
            throw new IllegalArgumentException("Chassis velocity must be complete or absent");
        long now = System.nanoTime();
        double loopMs = (now - previousNanos) / 1_000_000.0;
        previousNanos = now;
        double battery = hardwareMap.voltageSensor.iterator().next().getVoltage();
        StringBuilder row = new StringBuilder();
        row.append((now - startedNanos) / 1_000_000L).append(',').append(iteration++)
            .append(',').append(loopMs).append(',').append(battery);
        for (DcMotorEx motor : motors) row.append(',').append(motor.getPower())
            .append(',').append(motor.getCurrentPosition())
            .append(',').append(motor.getVelocity())
            .append(',').append(motor.getCurrent(CurrentUnit.AMPS));
        row.append(',').append(vxMps == null ? "" : vxMps)
            .append(',').append(vyMps == null ? "" : vyMps)
            .append(',').append(omegaRadS == null ? "" : omegaRadS);
        writer.write(row.toString()); writer.newLine();
        if (iteration % 20 == 0) writer.flush();
    }

    @Override public void close() throws IOException { writer.close(); }
}
