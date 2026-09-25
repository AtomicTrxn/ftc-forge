package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Turns in the physics renderer and reports IMU heading before and after reset. */
@Autonomous(name = "Turn and Reset IMU")
public class TurnAndResetOpMode extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        DcMotorEx lb = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        DcMotorEx rb = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(null));
        waitForStart();
        lf.setPower(-0.5);
        lb.setPower(-0.5);
        rf.setPower(0.5);
        rb.setPower(0.5);
        sleep(1000);
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
        sleep(100);
        telemetry.addData("Yaw before reset", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Yaw rate", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
        telemetry.update();
        imu.resetYaw();
        telemetry.addData("Yaw after reset", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }
}
