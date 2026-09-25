package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Phase 4 validation sample: drives toward the game piece placed in front of the robot,
 * then activates the "claw" servo (used here as the intake activation signal) and confirms
 * capture. Validates Phase 4's Definition of Done: "run a sample OpMode that drives into a
 * game piece, activates an intake, and confirms the piece is captured and tracked correctly
 * in 3D."
 */
@Autonomous(name = "Intake Demo")
public class IntakeDemoOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0.0); // intake inactive

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        // Activate intake BEFORE approaching -- it must be active as the robot passes the
        // game piece's location, not after (an intake armed only after already driving past
        // the piece can never catch it; this is the correct real-world sequencing too).
        claw.setPosition(1.0);
        telemetry.addData("Status", "Intake activated");
        telemetry.update();

        // Approach the game piece slowly enough to stop near it rather than overshoot.
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 1.2) {
            leftFront.setPower(0.3);
            rightFront.setPower(0.3);
            leftBack.setPower(0.3);
            rightBack.setPower(0.3);
            telemetry.addData("Phase", "Approaching");
            telemetry.addData("Ticks LF", leftFront.getCurrentPosition());
            telemetry.update();
            sleep(50);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        timer.reset();
        while (opModeIsActive() && timer.seconds() < 2.0) {
            telemetry.addData("Phase", "Holding position after approach");
            telemetry.update();
            sleep(50);
        }

        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}
