package com.acmerobotics.dashboard.telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

public class TelemetryPacket {
    private final Map<String, Object> data = new LinkedHashMap<>();
    private final StringBuilder lines = new StringBuilder();

    public void put(String key, Object value) {
        data.put(key, value);
    }

    public void addLine(String line) {
        lines.append(line).append('\n');
    }

    public Map<String, Object> getData() { return data; }
    public String getLines() { return lines.toString(); }
}
