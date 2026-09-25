package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/** Small iterative TeleOp that exercises the full FTC lifecycle in the desktop runner. */
@TeleOp(name = "Iterative Drive")
public class IterativeDriveOpMode extends OpMode {
    private DcMotorEx leftFront;
    private int loops;

    @Override public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        telemetry.addData("Status", "Iterative init");
        telemetry.update();
    }

    @Override public void start() { leftFront.setPower(0.3); }

    @Override public void loop() {
        loops++;
        if (loops >= 10) requestOpModeStop();
    }

    @Override public void stop() {
        leftFront.setPower(0);
        telemetry.addData("Status", "Iterative done; loops=" + loops);
        telemetry.update();
    }
}
