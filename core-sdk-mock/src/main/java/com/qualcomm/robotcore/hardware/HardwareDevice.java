package com.qualcomm.robotcore.hardware;

public interface HardwareDevice {
    String getDeviceName();
    String getConnectionInfo();
    void close();
}
