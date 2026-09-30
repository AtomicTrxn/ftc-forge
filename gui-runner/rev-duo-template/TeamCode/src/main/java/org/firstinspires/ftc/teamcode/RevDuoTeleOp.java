package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="RevDuoTeleOp")
public class RevDuoTeleOp extends OpMode {
    private DcMotorEx left,right,intake;
    public void init() {
        left=hardwareMap.get(DcMotorEx.class,"leftDrive"); right=hardwareMap.get(DcMotorEx.class,"rightDrive");
        intake=hardwareMap.get(DcMotorEx.class,"intake"); left.setDirection(DcMotor.Direction.REVERSE);
    }
    public void loop() {
        double forward=-gamepad1.left_stick_y*.5, turn=-gamepad1.right_stick_x*.35;
        double l=forward-turn, r=forward+turn, scale=Math.max(1,Math.max(Math.abs(l),Math.abs(r)));
        left.setPower(l/scale); right.setPower(r/scale); intake.setPower(gamepad1.a?.7:gamepad1.b?-.7:0);
        telemetry.addData("Controls","Arrows drive/turn; Space intake; E reverse; close window to stop"); telemetry.update();
    }
    public void stop() {left.setPower(0);right.setPower(0);intake.setPower(0);}
}
