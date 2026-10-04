package com.acmerobotics.dashboard.telemetry;

import com.acmerobotics.dashboard.canvas.Canvas;

import java.util.LinkedHashMap;
import java.util.Map;

public class TelemetryPacket {
    private final Map<String, Object> data = new LinkedHashMap<>();
    private final StringBuilder lines = new StringBuilder();
    private final Canvas fieldOverlay = new Canvas();

    public void put(String key, Object value) {
        data.put(key, value);
    }

    public void putAll(Map<String, ?> map) {
        data.putAll(map);
    }

    public void addLine(String line) {
        lines.append(line).append('\n');
    }

    public void clearLines() { lines.setLength(0); }

    /** Field-overlay drawing surface used by Road Runner-style pose and path visualisation. */
    public Canvas fieldOverlay() { return fieldOverlay; }

    public Map<String, Object> getData() { return data; }
    public String getLines() { return lines.toString(); }
}
