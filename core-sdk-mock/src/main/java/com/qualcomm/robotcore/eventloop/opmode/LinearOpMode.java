package com.qualcomm.robotcore.eventloop.opmode;

/**
 * Mock of com.qualcomm.robotcore.eventloop.opmode.LinearOpMode.
 * Per R1: runOpMode() runs on its own thread in the real SDK -- the executor replicates
 * that threading contract (see simrunner.Executor) so waitForStart()/opModeIsActive()
 * behave the same as on real hardware instead of deadlocking.
 */
public abstract class LinearOpMode extends OpMode {

    private volatile boolean started = false;
    private volatile boolean stopRequested = false;

    public abstract void runOpMode() throws InterruptedException;

    @Override public final void init() { }
    @Override public final void loop() { }

    public void waitForStart() throws InterruptedException {
        while (!started && !stopRequested) {
            Thread.sleep(10);
        }
    }

    public boolean opModeIsActive() { return started && !stopRequested; }
    public boolean isStopRequested() { return stopRequested || isStopRequestedInternal(); }
    public boolean isStarted() { return started; }
    public boolean opModeInInit() { return !started && !stopRequested; }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void idle() {
        Thread.yield();
    }

    /** Executor-facing hooks -- simulate the Driver Station's INIT/PLAY/STOP buttons. */
    public void internalSignalStart() { started = true; }
    public void internalSignalStop() { stopRequested = true; }
}
