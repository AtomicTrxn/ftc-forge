package simcore;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SimIMUTest {
    @Test void yawRateLatencyAndResetUseSimulationTime() {
        SimIMU imu = new SimIMU("imu");
        imu.setLatencyMs(10);
        imu.update(Math.toRadians(30), Math.PI, 0);
        imu.update(Math.toRadians(60), Math.PI / 2, 5);
        assertEquals(0, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 0.001);
        imu.update(Math.toRadians(90), Math.PI / 4, 10);
        assertEquals(30, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 0.001);
        assertEquals(180, imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate, 0.001);
        assertEquals(0, imu.getRobotYawPitchRollAngles().getAcquisitionTime());
        imu.resetYaw();
        assertEquals(0, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), 0.001);
        imu.update(Math.toRadians(120), Math.PI / 4, 20);
        assertEquals(60, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 0.001);
        assertEquals(Math.PI / 4, imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, 0.001);
        assertEquals(10_000_000L, imu.getRobotYawPitchRollAngles().getAcquisitionTime());
    }

    @Test void resetYawWrapsAtHalfTurn() {
        SimIMU imu = new SimIMU("imu");
        imu.setLatencyMs(0);
        imu.update(Math.toRadians(170), 0, 0);
        imu.resetYaw();
        imu.update(Math.toRadians(-170), 0, 20);
        assertEquals(20, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 0.001);
    }
}
