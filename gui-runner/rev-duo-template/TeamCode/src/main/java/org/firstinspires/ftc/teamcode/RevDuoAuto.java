package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="RevDuoAuto")
public class RevDuoAuto extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        DcMotorEx left=hardwareMap.get(DcMotorEx.class,"leftDrive"), right=hardwareMap.get(DcMotorEx.class,"rightDrive");
        DcMotorEx intake=hardwareMap.get(DcMotorEx.class,"intake");
        IMU imu=hardwareMap.get(IMU.class,"imu");
        left.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        intake.setPower(.7);
        left.setPower(.28); right.setPower(.28);
        sleep(1800);
        left.setPower(0); right.setPower(0);
        sleep(400);
        telemetry.addData("Drive ticks", left.getCurrentPosition()+", "+right.getCurrentPosition());
        telemetry.addData("Straight yaw",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Intake ticks",intake.getCurrentPosition()); telemetry.update();
        left.setPower(-.18); right.setPower(.18);
        sleep(1000);
        left.setPower(0); right.setPower(0);
        sleep(400);
        telemetry.addData("Turn yaw",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)); telemetry.update();
    }
}
