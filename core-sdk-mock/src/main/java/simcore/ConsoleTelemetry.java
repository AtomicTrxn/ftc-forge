package simcore;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/** Driver-Station-style telemetry -- Phase 1's "basic console logging" requirement. */
public class ConsoleTelemetry implements Telemetry {
    private final List<String> pending = new ArrayList<>();
    private boolean autoClear = true;

    @Override public Item addData(String caption, Object value) {
        pending.add(caption + " : " + value);
        return null;
    }

    @Override public Item addData(String caption, String format, Object... args) {
        pending.add(caption + " : " + String.format(format, args));
        return null;
    }

    @Override public Line addLine(String lineCaption) {
        pending.add(lineCaption);
        return null;
    }

    @Override public boolean update() {
        if (!pending.isEmpty()) {
            System.out.println("[TELEMETRY] " + String.join(" | ", pending));
        }
        if (autoClear) pending.clear();
        return true;
    }

    @Override public void clear() { pending.clear(); }
    @Override public void setAutoClear(boolean autoClear) { this.autoClear = autoClear; }
    @Override public void setMsTransmissionInterval(int ms) { }
    @Override public Log log() {
        return entry -> System.out.println("[LOG] " + entry);
    }
}
