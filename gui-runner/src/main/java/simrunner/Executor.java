package simrunner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import simcore.HardwareMapBuilder;

/**
 * Drives an OpMode's lifecycle (per R1's five-method contract) against a live HardwareMap.
 *
 * DEVIATION (documented): R3 specified a filtering parent/child classloader split so
 * simulator internals are invisible to team code. This build implements the simpler,
 * behaviorally-important half -- a fresh URLClassLoader per reload, so static team-code
 * state resets cleanly (see Main's reload path) -- but not the visibility-filtering half,
 * since a correct 3-tier loader graph is disproportionate risk for what Phase 1 validates.
 * Flagged here, not silently skipped; see Phase 1's RESULTS.md "Deviations" section.
 */
public class Executor {

    public static class RunResult {
        public volatile boolean completedNormally;
        public volatile boolean watchdogTriggered;
        public volatile Throwable failure;
    }

    /** A running (or finished) OpMode session -- lets a live render loop (Phase 2) poll state without blocking. */
    public static class Session {
        public final Thread opModeThread;
        public final RunResult result;
        Session(Thread opModeThread, RunResult result) {
            this.opModeThread = opModeThread;
            this.result = result;
        }
        public boolean isAlive() { return opModeThread.isAlive(); }
    }

    /** Starts either OpMode style on its own thread and returns without waiting for completion. */
    public static Session start(OpModeDiscovery.DiscoveredOpMode discovered,
                                 HardwareMap hardwareMap,
                                 Telemetry telemetry,
                                 Gamepad gamepad1, Gamepad gamepad2) throws Exception {
        OpMode instance = discovered.opModeClass.getDeclaredConstructor().newInstance();
        instance.hardwareMap = hardwareMap;
        instance.telemetry = telemetry;
        instance.gamepad1 = gamepad1;
        instance.gamepad2 = gamepad2;

        RunResult result = new RunResult();
        Thread opModeThread = new Thread(() -> {
            try {
                if (instance instanceof LinearOpMode linear) {
                    linear.runOpMode();
                } else {
                    runIterative(instance);
                }
                result.completedNormally = true;
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } catch (Throwable t) {
                result.failure = t;
            }
        }, "opmode-" + discovered.className);
        opModeThread.setDaemon(true);

        // Background motor-tick loop, standing in for a real fixed-timestep sim loop.
        Thread tickThread = new Thread(() -> {
            while (opModeThread.isAlive()) {
                HardwareMapBuilder.tickMotors(hardwareMap);
                try { Thread.sleep(20); } catch (InterruptedException e) { return; }
            }
        }, "motor-tick");
        tickThread.setDaemon(true);

        opModeThread.start();
        tickThread.start();

        // Headless stand-in for the Driver Station's INIT->PLAY transition. Iterative
        // OpModes perform their own short INIT period inside runIterative().
        if (instance instanceof LinearOpMode linear) {
            Thread.sleep(50);
            linear.internalSignalStart();
        }

        return new Session(opModeThread, result);
    }

    private static void runIterative(OpMode opMode) throws InterruptedException {
        try {
            opMode.init();
            long initDeadline = System.nanoTime() + 50_000_000L;
            do {
                opMode.init_loop();
                Thread.sleep(20);
            } while (!opMode.isStopRequestedInternal() && System.nanoTime() < initDeadline);
            if (!opMode.isStopRequestedInternal()) {
                opMode.start();
                opMode.resetRuntime();
                while (!opMode.isStopRequestedInternal()) {
                    long tickStart = System.nanoTime();
                    opMode.time = opMode.getRuntime();
                    opMode.loop();
                    long elapsedMs = (System.nanoTime() - tickStart) / 1_000_000L;
                    if (elapsedMs < 20) Thread.sleep(20 - elapsedMs);
                }
            }
        } finally {
            opMode.stop();
        }
    }

    /** Runs either OpMode style until normal completion or the watchdog fires. */
    public static RunResult runOpMode(OpModeDiscovery.DiscoveredOpMode discovered,
                                             HardwareMap hardwareMap,
                                             Telemetry telemetry,
                                             Gamepad gamepad1, Gamepad gamepad2,
                                             long watchdogTimeoutMillis) throws Exception {
        Session session = start(discovered, hardwareMap, telemetry, gamepad1, gamepad2);

        Watchdog watchdog = new Watchdog(session.opModeThread, watchdogTimeoutMillis);
        watchdog.start();
        session.result.watchdogTriggered = watchdog.isSessionDead();

        if (session.result.failure != null) {
            throw new RuntimeException("OpMode " + discovered.className + " threw", session.result.failure);
        }
        return session.result;
    }

    /** Compatibility alias for callers from Phase 1. */
    public static RunResult runLinearOpMode(OpModeDiscovery.DiscoveredOpMode discovered,
                                             HardwareMap hardwareMap, Telemetry telemetry,
                                             Gamepad gamepad1, Gamepad gamepad2,
                                             long watchdogTimeoutMillis) throws Exception {
        return runOpMode(discovered, hardwareMap, telemetry, gamepad1, gamepad2, watchdogTimeoutMillis);
    }
}
