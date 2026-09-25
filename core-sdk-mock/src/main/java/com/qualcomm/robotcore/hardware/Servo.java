package com.qualcomm.robotcore.hardware;

public interface Servo extends HardwareDevice {
    enum Direction { FORWARD, REVERSE }

    void setDirection(Direction direction);
    Direction getDirection();
    void setPosition(double position);
    double getPosition();
}
