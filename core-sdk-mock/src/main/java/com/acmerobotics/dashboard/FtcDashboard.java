package com.acmerobotics.dashboard;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.*;

/**
 * From-scratch shim for the real com.acmerobotics.dashboard.FtcDashboard (R1's findings):
 * same public API surface teams call (getInstance/getTelemetry/@Config scanning/TelemetryPacket),
 * without the real class's Android-Activity-lifecycle machinery.
 *
 * DEVIATION (documented, not silent): the real FtcDashboard is backed by DashboardCore, a
 * NanoHTTPD-based live web UI. Standing up a real embedded web server + the dashboard's
 * React frontend is out of scope for Phase 1's console-logging goal -- this shim proves
 * out the *API surface* (so real team code compiles and runs against it) by printing to
 * console instead of serving a browser UI. A real DashboardCore-backed server is a
 * reasonable Phase 2/3 follow-up, not required for Phase 1's Definition of Done.
 */
public class FtcDashboard {

    private static final FtcDashboard INSTANCE = new FtcDashboard();

    public static FtcDashboard getInstance() {
        return INSTANCE;
    }

    private final Set<Class<?>> configClasses = Collections.synchronizedSet(new LinkedHashSet<>());
    private final Telemetry telemetry = new DashboardTelemetry();

    private FtcDashboard() { }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void sendTelemetryPacket(TelemetryPacket packet) {
        if (!packet.getLines().isEmpty()) {
            System.out.print("[DASHBOARD] " + packet.getLines());
        }
        for (Map.Entry<String, Object> e : packet.getData().entrySet()) {
            System.out.println("[DASHBOARD] " + e.getKey() + " = " + e.getValue());
        }
    }

    /** Registers a class for @Config live-tuning. Real dashboards do this via classpath scan at startup. */
    public void registerConfigClass(Class<?> clazz) {
        if (clazz.isAnnotationPresent(Config.class)) {
            configClasses.add(clazz);
        }
    }

    /** Snapshot of every public static field across registered @Config classes -- proves the mechanism works. */
    public Map<String, Object> getConfigSnapshot() {
        Map<String, Object> snapshot = new LinkedHashMap<>();
        for (Class<?> clazz : configClasses) {
            for (Field f : clazz.getFields()) {
                if (Modifier.isStatic(f.getModifiers())) {
                    try {
                        snapshot.put(clazz.getSimpleName() + "." + f.getName(), f.get(null));
                    } catch (IllegalAccessException ignored) {
                    }
                }
            }
        }
        return snapshot;
    }

    /** Minimal Telemetry sink -- console output stands in for the real dashboard's web UI (see class javadoc). */
    private static class DashboardTelemetry implements Telemetry {
        private boolean autoClear = true;
        private final List<String> pendingLines = new ArrayList<>();

        @Override public Item addData(String caption, Object value) {
            pendingLines.add(caption + " : " + value);
            return null;
        }

        @Override public Item addData(String caption, String format, Object... args) {
            pendingLines.add(caption + " : " + String.format(format, args));
            return null;
        }

        @Override public Line addLine(String lineCaption) {
            pendingLines.add(lineCaption);
            return null;
        }

        @Override public boolean update() {
            for (String line : pendingLines) {
                System.out.println("[DASHBOARD] " + line);
            }
            if (autoClear) pendingLines.clear();
            return true;
        }

        @Override public void clear() { pendingLines.clear(); }
        @Override public void setAutoClear(boolean autoClear) { this.autoClear = autoClear; }
        @Override public void setMsTransmissionInterval(int ms) { }
        @Override public Log log() {
            return entry -> System.out.println("[DASHBOARD LOG] " + entry);
        }
    }
}
