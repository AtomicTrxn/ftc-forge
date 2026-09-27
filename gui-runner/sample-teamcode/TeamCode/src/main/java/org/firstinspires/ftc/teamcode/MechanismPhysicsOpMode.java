package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** Run with the URDF example to observe joint limits, contact stalls, and encoder feedback. */
@Autonomous(name = "Mechanism Physics")
public class MechanismPhysicsOpMode extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm_motor");
        DcMotorEx slide = hardwareMap.get(DcMotorEx.class, "slide_motor");
        waitForStart();
        arm.setPower(.4);
        slide.setPower(.4);
        sleep(2000);
        telemetry.addData("Arm ticks", arm.getCurrentPosition());
        telemetry.addData("Slide ticks", slide.getCurrentPosition());
        telemetry.addData("Slide current A", slide.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
        arm.setPower(0);
        slide.setPower(0);
    }
}
