package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Real, unmodified-style sample OpMode: 4-motor Mecanum chassis, drive forward then strafe,
 * reading back IMU/voltage/encoder state along the way. Validates Phase 1's Definition of
 * Done: "a real/realistic sample OpMode runs headlessly end-to-end with correct console
 * output."
 */
@Autonomous(name = "Basic Mecanum Auto")
public class BasicMecanumOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        // Drive forward for 1 second. Real OpMode loops are naturally paced by I2C round-trip
        // time (~45 Hz per R1/R5); this mock's hardware calls are instant, so an explicit
        // sleep stands in for that until Phase 3 models real hardware timing costs.
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 1.0) {
            leftFront.setPower(0.5);
            rightFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightBack.setPower(0.5);
            telemetry.addData("Phase", "Forward");
            telemetry.addData("Ticks LF", leftFront.getCurrentPosition());
            telemetry.addData("Battery V", hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
            sleep(50);
        }

        // Strafe right for 1 second (Mecanum: left_front/right_back forward, right_front/left_back reverse).
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 1.0) {
            leftFront.setPower(0.5);
            rightFront.setPower(-0.5);
            leftBack.setPower(-0.5);
            rightBack.setPower(0.5);
            telemetry.addData("Phase", "Strafe Right");
            telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
            telemetry.update();
            sleep(50);
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        telemetry.addData("Status", "Done. Final ticks LF=" + leftFront.getCurrentPosition());
        telemetry.update();
    }
}
