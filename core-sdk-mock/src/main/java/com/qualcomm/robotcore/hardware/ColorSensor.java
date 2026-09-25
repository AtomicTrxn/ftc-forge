package com.qualcomm.robotcore.hardware;

public interface ColorSensor extends HardwareDevice {
    int red();
    int green();
    int blue();
    int alpha();
}
