package com.qualcomm.hardware.lynx;

import com.qualcomm.robotcore.hardware.VoltageSensor;

/** Mock of com.qualcomm.hardware.lynx.LynxModule -- bulk-caching control hub/expansion hub stub. */
public class LynxModule implements VoltageSensor {

    public enum BulkCachingMode { OFF, AUTO, MANUAL }

    private final String name;
    private BulkCachingMode cachingMode = BulkCachingMode.OFF;
    private volatile double voltage = 12.6;

    public LynxModule(String name) { this.name = name; }

    public void setBulkCachingMode(BulkCachingMode mode) { this.cachingMode = mode; }
    public BulkCachingMode getBulkCachingMode() { return cachingMode; }
    public void clearBulkCache() { /* no-op in v1: no real bulk-read batching yet */ }

    /** Simulator hook: the shared battery-sag voltage. Like the real hub, the module doubles as a VoltageSensor. */
    public void setVoltage(double v) { this.voltage = v; }
    @Override public double getVoltage() { return voltage; }

    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Lynx Module \"" + name + "\""; }
    @Override public void close() { }
}
