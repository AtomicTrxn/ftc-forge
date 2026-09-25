package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Road-Runner-quickstart-style sample: imports FTCDashboard directly and uses a @Config
 * class for live-tunable constants, exactly like a real quickstart-based team's code.
 * Validates Phase 1's Definition of Done: "a Road-Runner-quickstart-style sample (importing
 * FTCDashboard) compiles and runs against the shim."
 */
@TeleOp(name = "Dashboard Drive")
public class DashboardOpMode extends LinearOpMode {

    @Config
    public static class Tuning {
        public static double DRIVE_POWER = 0.5;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.registerConfigClass(Tuning.class);

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "right_front_drive");

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 1.0) {
            leftFront.setPower(Tuning.DRIVE_POWER);
            rightFront.setPower(Tuning.DRIVE_POWER);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("drivePower", Tuning.DRIVE_POWER);
            packet.put("leftFrontTicks", leftFront.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("drivePower", Tuning.DRIVE_POWER);
            telemetry.update();
            sleep(50);
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        telemetry.addData("Status", "Dashboard OpMode done. Config snapshot=" + dashboard.getConfigSnapshot());
        telemetry.update();
    }
}
