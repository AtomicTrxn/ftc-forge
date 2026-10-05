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

    @Test void fullPoseAndBodyRatesRetainLatencyUnitsAndYawOnlyReset() {
        var imu=new SimIMU("imu");imu.setLatencyMs(10);imu.update(.4,-.2,.3,1,2,3,0);imu.update(.8,-.5,.7,4,5,6,10);
        var pose=imu.getRobotYawPitchRollAngles();assertEquals(-.2,pose.getPitch(AngleUnit.RADIANS),1e-9);assertEquals(.3,pose.getRoll(AngleUnit.RADIANS),1e-9);
        var rate=imu.getRobotAngularVelocity(AngleUnit.RADIANS);assertEquals(1,rate.xRotationRate,1e-6);assertEquals(2,rate.yRotationRate,1e-6);assertEquals(3,rate.zRotationRate,1e-6);
        assertEquals(180/Math.PI,imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate,1e-4);imu.resetYaw();assertEquals(0,imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS),1e-9);assertEquals(.3,imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS),1e-9);
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
