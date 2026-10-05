package physics;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
class TireResponseTest {
    private final TireFriction.Spec tire=new TireFriction.Spec(.9,.7,.1,100,.15);
    @Test void zeroRelaxationPreservesLegacyContactLaw() {
        for(double slip:new double[]{-5,-.1,0,.1,5})for(double lateral:new double[]{-3,0,3}) {
            var expected=TireFriction.solve(tire,slip,lateral,10,1./120,1,1);
            var actual=new TireResponse().solve(tire,TireResponse.Spec.steady(tire),slip,lateral,10,1./120,1,1);
            assertEquals(expected.longitudinalN(),actual.longitudinalN(),1e-12);assertEquals(expected.lateralN(),actual.lateralN(),1e-12);
        }
    }
    @Test void forceBuildupMatchesTimeConstantAcrossTimestepsAndIndependentLateralGrip() {
        var response=new TireResponse.Spec(60,.5,.1);
        double previous=0;
        for(int hz:new int[]{30,120,480}) {
            var state=new TireResponse();TireFriction.Force f=null;
            for(int i=0;i<hz;i++)f=state.solve(tire,response,.02,.02,100,1./hz,0,0);
            assertEquals(2*(1-Math.exp(-10)),f.longitudinalN(),1e-10);assertEquals(-1.2*(1-Math.exp(-10)),f.lateralN(),1e-10);
            if(previous!=0)assertEquals(previous,f.lateralN(),1e-10);previous=f.lateralN();
        }
        var f=new TireResponse().solve(tire,new TireResponse.Spec(1000,.05,0),0,10,10,.01,0,0);
        assertEquals(-.7/.9*.05*10,f.lateralN(),1e-10);
    }
    @Test void changingSolvedLoadScalesUtilizationWithoutFilteringLoadTwice() {
        var state=new TireResponse();var response=new TireResponse.Spec(100,.5,.1);double dt=.01;
        for(int i=1;i<=100;i++){double load=i%2==0?20:10;var f=state.solve(tire,response,10,0,load,dt,0,0);assertEquals(.7*load*(1-Math.exp(-i*.1)),f.longitudinalN(),1e-10);}
        state.reset();assertEquals(.7*10*(1-Math.exp(-.1)),state.solve(tire,response,10,0,10,dt,0,0).longitudinalN(),1e-10);
        var high=new TireFriction.Spec(3,2,1,200000,.2);var expected=TireFriction.solve(high,.1,.2,10,.01,0,0);var actual=new TireResponse().solve(high,TireResponse.Spec.steady(high),.1,.2,10,.01,0,0);assertEquals(expected.longitudinalN(),actual.longitudinalN(),1e-12);assertEquals(expected.lateralN(),actual.lateralN(),1e-12);
    }
    @Test void reversalLoadLossAndMixedSlipCannotInjectEnergyOrExceedImpulseBudget() {
        var state=new TireResponse();var response=new TireResponse.Spec(500,.8,.2);
        for(int i=0;i<400;i++) {
            double slip=Math.sin(i*.17),lateral=Math.cos(i*.13),load=i%7==0?0:10;
            var f=state.solve(tire,response,slip,lateral,load,1./120,2,3);
            assertTrue(-f.longitudinalN()*slip+f.lateralN()*lateral<=1e-12);
            assertTrue(Math.abs(f.longitudinalN())<=Math.abs(slip)/(1./120*2)+1e-12);
            assertTrue(Math.abs(f.lateralN())<=Math.abs(lateral)/(1./120*3)+1e-12);
            assertTrue(Math.hypot(f.longitudinalN(),f.lateralN())<=9+1e-12);
            if(load==0)assertEquals(new TireFriction.Force(0,0,false),f);
        }
        assertThrows(IllegalArgumentException.class,()->new TireResponse.Spec(10,.5,Double.NaN));
        assertThrows(IllegalArgumentException.class,()->new TireResponse.Spec(-1,.5,.1));
    }
}
