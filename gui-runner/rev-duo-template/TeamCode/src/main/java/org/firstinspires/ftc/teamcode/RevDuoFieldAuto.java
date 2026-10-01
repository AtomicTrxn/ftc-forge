package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/** Exercise the same CAD robot on a field selected independently at launch. */
@Autonomous(name="RevDuoFieldAuto")
public class RevDuoFieldAuto extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        DcMotor left=hardwareMap.get(DcMotor.class,"leftDrive"),right=hardwareMap.get(DcMotor.class,"rightDrive");
        left.setDirection(DcMotor.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();left.setPower(.1);right.setPower(.1);sleep(1500);
        left.setPower(-.1);right.setPower(.1);sleep(1000);
        left.setPower(.1);right.setPower(.1);sleep(1000);
        left.setPower(0);right.setPower(0);sleep(2500);
        telemetry.addData("Field demo","drive, turn and settle complete");telemetry.update();
    }
}
