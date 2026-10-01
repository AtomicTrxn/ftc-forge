package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/** Short physical drive demonstration, usable with either field and either piece mode. */
@Autonomous(name="FieldDriveOpMode")
public class FieldDriveOpMode extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        DcMotor lf=hardwareMap.get(DcMotor.class,"left_front_drive"),rf=hardwareMap.get(DcMotor.class,"right_front_drive"),
            lb=hardwareMap.get(DcMotor.class,"left_back_drive"),rb=hardwareMap.get(DcMotor.class,"right_back_drive");
        rf.setDirection(DcMotor.Direction.REVERSE);rb.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();lf.setPower(.15);rf.setPower(.15);lb.setPower(.15);rb.setPower(.15);sleep(1500);
        lf.setPower(0);rf.setPower(0);lb.setPower(0);rb.setPower(0);sleep(4500);
        telemetry.addData("Field demo","drive and settle complete");telemetry.update();
    }
}
