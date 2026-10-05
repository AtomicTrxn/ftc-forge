package simrunner;

import com.qualcomm.robotcore.hardware.Gamepad;
import simcore.ConsoleTelemetry;
import java.net.URLClassLoader;
import java.nio.file.*;
import java.security.MessageDigest;
import java.util.*;
import java.util.concurrent.locks.LockSupport;

/** Wall-paced TeamCode with one fixed-step native/motor clock. One active run per JVM. */
public final class HeadlessPhysicsRunner {
    public record Options(double durationSeconds, long watchdogMillis, double settleSeconds, long maxLagMillis) {
        public Options {
            if (!Double.isFinite(durationSeconds) || durationSeconds <= 0 || durationSeconds > 600
                    || watchdogMillis < 1 || watchdogMillis > 605_000
                    || !Double.isFinite(settleSeconds) || settleSeconds < 0 || settleSeconds > 5
                    || maxLagMillis < 1 || maxLagMillis > 5000)
                throw new IllegalArgumentException("Require duration >0..600s, watchdog 1..605000ms, settle 0..5s, max lag 1..5000ms");
        }
        public static Options defaults() { return new Options(10, 12_000, .25, 250); }
    }
    private static final int MAX_SAMPLES = 10_000;
    private static Executor.Session orphan;

    /** Writes evidence even for startup failure; unsuccessful runs return success=false. */
    public static synchronized Map<String,Object> run(Path project, String requested, Path output, Options options) throws Exception {
        var report = new LinkedHashMap<String,Object>();
        report.put("schema_version", 1); report.put("project", project.toAbsolutePath().toString()); report.put("opmode", requested);
        report.put("engine", "Minie 9.0.3 / Bullet"); report.put("renderer", false);
        report.put("units", Map.of("xyz", "m, FTC x forward / y left / z up", "yaw", "rad", "time", "s", "encoder", "ticks"));
        report.put("timing_policy", "Wall-paced, fixed native steps. SDK timers/sleeps use wall time; scheduling is not deterministic. Duration includes INIT. Watchdog excludes compilation, construction and settling.");
        report.put("limits", Map.of("duration_s", options.durationSeconds, "watchdog_ms", options.watchdogMillis,
            "settle_s", options.settleSeconds, "max_lag_ms", options.maxLagMillis));
        report.put("assumptions", List.of("Uses configured geometry, collisions, motor/calibration and field profiles; generated defaults are not measured hardware.",
            "Generic chassis remains a box with aggregate drive. Imported native wheels/tires/mechanisms use their configured models.",
            "Scene sensors are geometric observations, not rendered camera frames. Field rules are configured practice rules, not an official referee.",
            "Stop/interrupt-ignoring code is a daemon; run the CLI in its own JVM for isolation. No physics/motor ticks continue after stop."));
        report.put("outcome", "startup_failure"); report.put("success", false);
        Executor.Session session = null;
        HeadlessScene scene = null;
        URLClassLoader loader = null;
        var samples = new ArrayList<Map<String,Object>>();
        boolean trajectoryTruncated = false;
        try {
            if (orphan != null && orphan.isAlive()) throw new IllegalStateException("Previous TeamCode still ignores stop. Restart the runner JVM before another run.");
            orphan = null;
            var config = SimConfig.load(project);
            report.put("inputs", inputs(project, config));
            var classes = TeamCodeCompiler.compile(project, project.resolve(config.sourceRoot), config.extraClasspath);
            loader = TeamCodeCompiler.newClassLoader(project, classes, config.extraClasspath);
            var target = OpModeDiscovery.discover(classes, loader).stream().filter(d -> d.className.equals(requested)
                || d.className.endsWith("." + requested) || d.displayName.equals(requested)).findFirst()
                .orElseThrow(() -> new IllegalArgumentException("No discovered OpMode matches " + requested));
            report.put("opmode_class", target.className);
            report.put("sensor_settings", config.sensors.stream().map(s -> Map.of("name", s.name(), "type", s.type(), "latency_s", s.latencyS(), "update_hz", s.updateHz())).toList());
            com.jme3.system.NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
            scene = new HeadlessScene(project, config);
            report.put("scene", scene.description);
            double dt = scene.space.getAccuracy();
            report.put("fixed_step_s", dt);
            int settleTicks = (int)Math.ceil(options.settleSeconds / dt);
            for (int i = 0; i < settleTicks; i++) scene.tick(false);
            scene.publishPose(); // first odometry origin is the settled native pose
            report.put("initial", scene.sample(0));
            long started = System.nanoTime();
            session = Executor.startExternallyDriven(target, scene.hardware, new ConsoleTelemetry(), new Gamepad(), new Gamepad());
            int ticks = 0;
            double maxLag = 0, nextSample = 0;
            boolean played = false;
            String outcome;
            for (;;) {
                long now = System.nanoTime();
                double wallSeconds = (now - started) / 1e9;
                if (Thread.currentThread().isInterrupted()) { outcome = "interrupted"; break; }
                if (!session.isAlive()) { outcome = session.result.failure == null && session.result.completedNormally ? "completed" : "opmode_failure"; break; }
                if (wallSeconds >= options.watchdogMillis / 1000.) { outcome = "watchdog"; session.result.watchdogTriggered = true; break; }
                if (ticks * dt >= options.durationSeconds) { outcome = "duration_complete"; break; }
                long deadline = started + Math.round((ticks + 1) * dt * 1e9);
                if (now < deadline) { LockSupport.parkNanos(Math.min(deadline - now, 2_000_000)); continue; }
                double lag = (now - deadline) / 1e6;
                maxLag = Math.max(maxLag, lag);
                if (lag > options.maxLagMillis) { outcome = "timing_overrun"; break; }
                if (!played && wallSeconds >= .05) {
                    session.signalStart(); played = true;
                    if (scene.behavior != null) scene.behavior.toggle();
                }
                scene.tick(); ticks++;
                if (ticks * dt >= nextSample) {
                    if (samples.size() < MAX_SAMPLES) samples.add(scene.sample(ticks * dt));
                    else trajectoryTruncated = true;
                    nextSample += .05;
                }
            }
            report.put("outcome", outcome);
            report.put("success", outcome.equals("completed") || outcome.equals("duration_complete"));
            report.put("timing", Map.of("physics_ticks", ticks, "simulation_s", ticks * dt,
                "settled_native_s", settleTicks * dt, "wall_s", (System.nanoTime() - started) / 1e9, "max_lag_ms", maxLag));
            // Capture commanded effort before stop zeroes hardware. Both states are labeled.
            report.put("final", scene.sample(ticks * dt));
            report.put("diagnostics", scene.diagnostics());
            report.put("sensor_summaries", scene.sensors.summaries());
            if (scene.behavior != null) report.put("field_behavior", scene.behavior.report());
            if (session.result.failure != null) report.put("failure", describe(session.result.failure));
        } catch (Exception | LinkageError e) {
            report.put("outcome", scene == null ? "startup_failure" : "simulation_failure");
            report.put("success", false); report.put("failure", describe(e));
        } finally {
            // Stop before destroying native bodies or closing the TeamCode loader.
            if (session != null) {
                session.requestStop();
                try { session.opModeThread.join(100); }
                catch (InterruptedException e) { Thread.currentThread().interrupt(); }
                report.put("opmode_thread_terminated", !session.isAlive());
                if (session.result.failure != null) {
                    report.put("failure", describe(session.result.failure));
                    if (Boolean.TRUE.equals(report.get("success"))) report.put("outcome", "opmode_failure");
                    report.put("success", false);
                }
                if (session.isAlive()) orphan = session;
                if (session.isAlive() && Boolean.TRUE.equals(report.get("success"))) { report.put("success", false); report.put("outcome", "stop_timeout"); }
            }
            if (scene != null) {
                for (var m : scene.hardware.getAll(com.qualcomm.robotcore.hardware.DcMotor.class)) m.setPower(0);
                for (var m : scene.hardware.getAll(com.qualcomm.robotcore.hardware.CRServo.class)) m.setPower(0);
                report.put("stop_commands_issued", true);
                report.put("motors_stopped", session == null || !session.isAlive());
                try { scene.close(); } catch (Exception | LinkageError e) { cleanupFailure(report, e); }
            }
            if (loader != null) try { loader.close(); } catch (Exception e) { cleanupFailure(report, e); }
            report.put("trajectory", samples);
            report.put("trajectory_truncated", trajectoryTruncated);
            ProfileIO.save(output, report);
        }
        return report;
    }

    private static void cleanupFailure(Map<String,Object> report, Throwable e) {
        if (Boolean.TRUE.equals(report.get("success"))) report.put("outcome", "cleanup_failure");
        report.put("success", false); report.put("cleanup_failure", describe(e));
    }
    private static String describe(Throwable e) {
        String s = e.toString();
        if (e.getCause() != null) s += "; cause: " + e.getCause();
        return s.substring(0, Math.min(4000, s.length()));
    }
    private static Map<String,Object> inputs(Path project, SimConfig config) throws Exception {
        var out = new LinkedHashMap<String,Object>();
        for (String path : Arrays.asList("sim.config", config.robotConfig, config.presetMotors, config.urdf, config.calibration,
                config.field.packagePath())) if (path != null) {
            Path file = project.resolve(path).toRealPath();
            out.put(file.toString(), HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(Files.readAllBytes(file))));
        }
        if (config.robotProfile != null) {
            Path profile = config.robotProfile.path;
            out.put(profile.toString(), HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(Files.readAllBytes(profile))));
            out.put("robot_runtime", config.robotProfile.runtime);
            out.put("robot_parameters", config.robotProfile.parameters);
        }
        out.put("field", Map.of("source", config.field.source(), "mode", config.field.mode(), "piece_set", config.field.pieceSet()));
        out.put("imu_latency_ms", config.imuLatencyMs);
        return out;
    }
}
