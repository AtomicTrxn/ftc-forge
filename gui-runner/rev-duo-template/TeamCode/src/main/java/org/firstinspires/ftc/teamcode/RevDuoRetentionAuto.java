package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/** Demonstrates the simulator's opt-in, uncalibrated torus retention profile. */
@Autonomous(name="RevDuoRetentionAuto")
public class RevDuoRetentionAuto extends LinearOpMode {
    protected boolean releaseAfterCarry(){return false;}
    public void runOpMode() throws InterruptedException {
        DcMotorEx left=hardwareMap.get(DcMotorEx.class,"leftDrive"),right=hardwareMap.get(DcMotorEx.class,"rightDrive");
        DcMotorEx intake=hardwareMap.get(DcMotorEx.class,"intake");
        left.setDirection(DcMotor.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        intake.setPower(1);left.setPower(.1);right.setPower(.1);sleep(1200);
        left.setPower(0);right.setPower(0);sleep(4000);
        left.setPower(.12);right.setPower(.12);sleep(1000);
        left.setPower(-.12);right.setPower(.12);sleep(1000);
        left.setPower(0);right.setPower(0);intake.setPower(0);sleep(2000);
        telemetry.addData("Retention demonstration","capture, carry, turn and stopped intake complete");
        telemetry.addData("Intake encoder",intake.getCurrentPosition());telemetry.update();
        if(releaseAfterCarry()){intake.setPower(-1);sleep(2000);intake.setPower(0);sleep(500);}
    }
}
