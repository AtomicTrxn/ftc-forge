package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Compatibility fixture written in the structure of a Pedro Pathing follower: an iterative
 * OpMode that configures a Pinpoint localizer the way PinpointConstants does (offsets, pod
 * resolution, encoder directions, resetPosAndIMU), reads its pose every loop, and also reads
 * an OTOS in inches/degrees. It does not use the Pedro jars; see gui-runner/COMPATIBILITY.md.
 */
@TeleOp(name = "Pedro Style Localizer")
public class PedroStyleOpMode extends OpMode {

    @Config
    public static class Tuning {
        public static double FORWARD_POD_Y_MM = 0, STRAFE_POD_X_MM = -101.6;
    }

    private GoBildaPinpointDriver pinpoint;
    private SparkFunOTOS otos;
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private Pose2D lastPose;
    private int loops;

    @Override public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(Tuning.STRAFE_POD_X_MM, Tuning.FORWARD_POD_Y_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 9, 8, AngleUnit.RADIANS, 0));

        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
        otos.calibrateImu();
        otos.resetTracking();
    }

    @Override public void loop() {
        loops++;
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.clearBulkCache();
        pinpoint.update();
        lastPose = pinpoint.getPosition();
        SparkFunOTOS.Pose2D otosPose = otos.getPosition();

        double power = 0.3;
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);

        telemetry.addData("pinpoint x (in)", lastPose.getX(DistanceUnit.INCH));
        telemetry.addData("pinpoint y (in)", lastPose.getY(DistanceUnit.INCH));
        telemetry.addData("pinpoint heading (rad)", lastPose.getHeading(AngleUnit.RADIANS));
        telemetry.addData("otos x (in)", otosPose.x);
        telemetry.update();
        if (loops >= 5) requestOpModeStop();
    }

    @Override public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        telemetry.addData("PEDRO_STYLE", "done loops=" + loops + " startX_in="
            + (lastPose == null ? "none" : lastPose.getX(DistanceUnit.INCH)));
        telemetry.update();
    }
}
