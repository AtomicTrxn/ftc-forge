package com.qualcomm.robotcore.eventloop.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Mock of com.qualcomm.robotcore.eventloop.opmode.OpMode.
 * The five lifecycle methods below are what Phase 1's executor schedules (per R1).
 */
public abstract class OpMode {
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public Gamepad gamepad1 = new Gamepad();
    public Gamepad gamepad2 = new Gamepad();
    public double time;

    private final ElapsedTime runtime = new ElapsedTime();
    private volatile boolean stopRequestedInternal = false;

    public abstract void init();
    public void init_loop() { }
    public void start() { }
    public abstract void loop();
    public void stop() { }

    public double getRuntime() { return runtime.seconds(); }
    public void resetRuntime() { runtime.reset(); }

    /** Executor-facing hook; not part of the documented public SDK surface, but harmless to expose. */
    public void requestOpModeStop() { stopRequestedInternal = true; }
    public boolean isStopRequestedInternal() { return stopRequestedInternal; }
}
