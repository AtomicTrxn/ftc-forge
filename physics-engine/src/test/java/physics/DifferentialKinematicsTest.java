package physics;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
class DifferentialKinematicsTest {
    @Test void forwardReverseArcAndPivotNeverCommandStrafe() {
        var k=new DifferentialKinematics(.4);
        var forward=k.forwardFromWheelSpeeds(1,1);
        assertEquals(1,forward.vx); assertEquals(0,forward.omega);
        assertEquals(-1,k.forwardFromWheelSpeeds(-1,-1).vx);
        var pivot=k.forwardFromWheelSpeeds(-1,1);
        assertEquals(0,pivot.vx); assertEquals(5,pivot.omega);
        var arc=k.forwardFromWheelSpeeds(.5,1);
        assertEquals(.75,arc.vx); assertEquals(1.25,arc.omega);
        for(var v:new MecanumKinematics.ChassisVelocity[]{forward,pivot,arc})assertEquals(0,v.vy);
    }
    @Test void rejectsDegenerateTrack() {
        for(double v:new double[]{0,-1,Double.NaN,Double.POSITIVE_INFINITY})
            assertThrows(IllegalArgumentException.class,()->new DifferentialKinematics(v));
    }
}
