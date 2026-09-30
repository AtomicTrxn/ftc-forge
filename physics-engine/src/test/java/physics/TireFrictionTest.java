package physics;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
class TireFrictionTest {
    @Test void frictionEllipseBoundsAndDissipatesAtEverySlipDirection() {
        for(double lateralScale:new double[]{0,.05,1}) {
            var spec=new TireFriction.Spec(.9,.7,lateralScale,100,.15);
            for(double slip:new double[]{-10,-.1,0,.1,10})for(double lateral:new double[]{-5,-.1,0,.1,5}) {
                var f=TireFriction.solve(spec,slip,lateral,10,1.0/120,1,1);
                assertTrue(Math.hypot(f.longitudinalN()/9,lateralScale==0?0:f.lateralN()/(9*lateralScale))<=1.000001);
                assertTrue(-f.longitudinalN()*slip+f.lateralN()*lateral<=1e-12,"Contact must dissipate relative kinetic energy");
                if(lateralScale==0)assertEquals(0,f.lateralN(),0);
            }
        }
    }
    @Test void unsupportedWheelCannotProduceForceAndLowGripSlides() {
        var s=new TireFriction.Spec(.1,.08,1,100,.15);
        assertEquals(new TireFriction.Force(0,0,false),TireFriction.solve(s,10,2,0,.01,1,1));
        var f=TireFriction.solve(s,10,0,10,.01,1,1);
        assertEquals(.8,f.longitudinalN(),1e-8);assertTrue(f.sliding());
        assertThrows(IllegalArgumentException.class,()->new TireFriction.Spec(.1,.2,1,100,.1));
        assertThrows(IllegalArgumentException.class,()->new TireFriction.Spec(Double.NaN,.2,1,100,.1));
    }
}
