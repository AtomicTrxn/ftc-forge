package com.qualcomm.hardware.lynx;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/** Mock of com.qualcomm.hardware.lynx.LynxModule -- bulk-caching control hub/expansion hub stub. */
public class LynxModule implements HardwareDevice {

    public enum BulkCachingMode { OFF, AUTO, MANUAL }

    private final String name;
    private BulkCachingMode cachingMode = BulkCachingMode.OFF;

    public LynxModule(String name) { this.name = name; }

    public void setBulkCachingMode(BulkCachingMode mode) { this.cachingMode = mode; }
    public BulkCachingMode getBulkCachingMode() { return cachingMode; }
    public void clearBulkCache() { /* no-op in v1: no real bulk-read batching yet */ }

    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Lynx Module \"" + name + "\""; }
    @Override public void close() { }
}
