package simcore;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.junit.jupiter.api.Test;
import physics.MotorSpec;

import static org.junit.jupiter.api.Assertions.*;

class ExternalMotorTest {
    @Test void reversedShaftFeedbackAndResetKeepLogicalEncoderState() {
        SimDcMotorEx motor = new SimDcMotorEx("joint", new MotorSpec("test", 1, 2, 9, 30, 12, 500));
        motor.useExternalShaft();
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.syncExternalShaft(-Math.PI, -15);
        motor.integrate(12, .02, 0);
        motor.integrate(12, .02, 20);
        assertEquals(250, motor.getCurrentPosition());
        assertEquals(15 * 500 / (2 * Math.PI), motor.getVelocity(), .001);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(.5);
        assertEquals(.5, motor.commandedPower(), .001);
        assertEquals(-.5, motor.signedCommandedPower(), .001);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.syncExternalShaft(-Math.PI, 0);
        motor.integrate(12, .02, 40);
        motor.integrate(12, .02, 60);
        assertEquals(0, motor.getCurrentPosition());
        motor.syncExternalShaft(-2 * Math.PI, 0);
        motor.integrate(12, .02, 80);
        motor.integrate(12, .02, 100);
        assertEquals(250, motor.getCurrentPosition());
        assertEquals(-1, motor.externalShaftTorque(), .001);
    }
}
