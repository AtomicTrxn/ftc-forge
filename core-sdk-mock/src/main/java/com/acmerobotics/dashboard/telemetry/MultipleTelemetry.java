package com.acmerobotics.dashboard.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Mirrors telemetry to several Telemetry sinks at once (e.g. Driver Station + dashboard). */
public class MultipleTelemetry implements Telemetry {
    private final Telemetry[] telemetries;

    public MultipleTelemetry(Telemetry... telemetries) {
        this.telemetries = telemetries;
    }

    @Override public Item addData(String caption, Object value) {
        Item item = null;
        for (Telemetry t : telemetries) item = t.addData(caption, value);
        return item;
    }

    @Override public Item addData(String caption, String format, Object... args) {
        Item item = null;
        for (Telemetry t : telemetries) item = t.addData(caption, format, args);
        return item;
    }

    @Override public Line addLine(String lineCaption) {
        Line line = null;
        for (Telemetry t : telemetries) line = t.addLine(lineCaption);
        return line;
    }

    @Override public boolean update() {
        boolean ok = true;
        for (Telemetry t : telemetries) ok &= t.update();
        return ok;
    }

    @Override public void clear() { for (Telemetry t : telemetries) t.clear(); }
    @Override public void setAutoClear(boolean autoClear) { for (Telemetry t : telemetries) t.setAutoClear(autoClear); }
    @Override public void setMsTransmissionInterval(int ms) { for (Telemetry t : telemetries) t.setMsTransmissionInterval(ms); }
    @Override public Log log() { return telemetries.length > 0 ? telemetries[0].log() : null; }
}
