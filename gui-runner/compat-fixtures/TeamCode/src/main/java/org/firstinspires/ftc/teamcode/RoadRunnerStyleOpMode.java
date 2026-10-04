package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Compatibility fixture written in the structure of the Road Runner quickstart's MecanumDrive
 * and tuning OpModes, using only the FTC SDK and FtcDashboard calls that code makes: hub bulk
 * caching, the hub voltage sensor for feed-forward, IMU initialization with a hub orientation,
 * encoder reads, a MultipleTelemetry sink, and TelemetryPacket field-overlay drawing.
 * It does not use the Road Runner jars; see gui-runner/COMPATIBILITY.md.
 */
@Autonomous(name = "RR Style Drive")
public class RoadRunnerStyleOpMode extends LinearOpMode {

    @Config
    public static class Params {
        public static double kV = 0.0002, kS = 0.05, TARGET_POWER = 0.4;
    }

    private static class MecanumDrive {
        final DcMotorEx leftFront, leftBack, rightBack, rightFront;
        final VoltageSensor voltageSensor;
        final IMU lazyImu;

        MecanumDrive(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
            leftFront = hardwareMap.get(DcMotorEx.class, "left_front_drive");
            leftBack = hardwareMap.get(DcMotorEx.class, "left_back_drive");
            rightBack = hardwareMap.get(DcMotorEx.class, "right_back_drive");
            rightFront = hardwareMap.get(DcMotorEx.class, "right_front_drive");
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);
            lazyImu = hardwareMap.get(IMU.class, "imu");
            lazyImu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }

        void setDrivePowers(double forward) {
            double compensation = 12.0 / voltageSensor.getVoltage();
            double power = Math.min(1.0, (Params.kS + forward) * compensation);
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightBack.setPower(power);
            rightFront.setPower(power);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multi = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().registerConfigClass(Params.class);
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        waitForStart();

        int overlayCalls = 0;
        long deadline = System.currentTimeMillis() + 700;
        while (opModeIsActive() && System.currentTimeMillis() < deadline) {
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) module.clearBulkCache();
            drive.setDrivePowers(Params.TARGET_POWER);

            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();
            canvas.setStroke("#3F51B5").strokeCircle(0, 0, 9).strokeLine(0, 0, 9, 0);
            canvas.strokePolyline(new double[]{0, 12, 24}, new double[]{0, 6, 0});
            overlayCalls = canvas.getOperationCount();
            packet.put("leftFrontTicks", drive.leftFront.getCurrentPosition());
            packet.put("leftFrontVelocity", drive.leftFront.getVelocity());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            multi.addData("voltage", drive.voltageSensor.getVoltage());
            multi.addData("heading", drive.lazyImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            multi.update();
            sleep(20);
        }
        drive.setDrivePowers(-Params.kS);
        telemetry.addData("RR_STYLE", "done overlayCalls=" + overlayCalls
            + " ticks=" + drive.leftFront.getCurrentPosition());
        telemetry.update();
    }
}
