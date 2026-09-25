package com.qualcomm.robotcore.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Mock of the unified IMU interface (real SDK since 8.1, supports both BHI260AP and BNO055).
 */
public interface IMU extends HardwareDevice {

    class Parameters {
        public final RevHubOrientationOnRobot orientationOnRobot;
        public Parameters(RevHubOrientationOnRobot orientationOnRobot) {
            this.orientationOnRobot = orientationOnRobot;
        }
    }

    boolean initialize(Parameters parameters);
    YawPitchRollAngles getRobotYawPitchRollAngles();
    AngularVelocity getRobotAngularVelocity(AngleUnit unit);
    void resetYaw();
}
