package physics;

/** Dissipative first-order contact utilization response with independently specified lateral behavior. */
public final class TireResponse {
    public record Spec(double lateralStiffnessNPerMps,double lateralMu,double relaxationTimeS) {
        public Spec {
            if(!Double.isFinite(lateralStiffnessNPerMps)||lateralStiffnessNPerMps<0
                ||!Double.isFinite(lateralMu)||lateralMu<0
                ||!Double.isFinite(relaxationTimeS)||relaxationTimeS<0||relaxationTimeS>2)
                throw new IllegalArgumentException("Tire response requires nonnegative lateral stiffness N/(m/s) and lateral mu, relaxation 0..2 s");
        }
        public static Spec steady(TireFriction.Spec tire){return new Spec(tire.stiffnessNPerMps()*tire.lateralScale(),tire.staticMu()*tire.lateralScale(),0);}
    }
    private double longitudinal,lateral;
    public void reset(){longitudinal=lateral=0;}
    public TireFriction.Force solve(TireFriction.Spec tire,Spec response,double slip,double lateralSpeed,double normal,
                                    double dt,double inverseLongMass,double inverseLatMass) {
        // Validate and preserve the exact legacy law when response parameters are unchanged.
        var legacy=TireFriction.solve(tire,slip,lateralSpeed,normal,dt,inverseLongMass,inverseLatMass);
        if(normal==0)return legacy; // No solved load this step; caller resets when native support is lost.
        double mu=TireFriction.coefficient(tire,Math.hypot(slip,lateralSpeed));
        double capX=mu*normal,capY=mu/tire.staticMu()*response.lateralMu()*normal;
        double targetX=tire.stiffnessNPerMps()*slip/(1+tire.stiffnessNPerMps()*dt*inverseLongMass);
        double ky=response.lateralStiffnessNPerMps();
        double targetY=capY==0?0:-ky*lateralSpeed/(1+ky*dt*inverseLatMass);
        double ellipse=Math.hypot(targetX/capX,capY==0?0:targetY/capY);boolean sliding=ellipse>1;
        if(sliding){targetX/=ellipse;targetY/=ellipse;}
        // Relax normalized contact utilization. Native impulses vary with timestep; filtering
        // their force magnitude would count the changing normal load twice and attenuate grip.
        double alpha=response.relaxationTimeS()==0?1:-Math.expm1(-dt/response.relaxationTimeS());
        longitudinal+=alpha*(targetX/capX-longitudinal);
        lateral+=alpha*((capY==0?0:targetY/capY)-lateral);
        if(longitudinal*slip<=0)longitudinal=0;
        if(lateral*lateralSpeed>=0)lateral=0;
        double actualEllipse=Math.hypot(longitudinal,lateral);
        if(actualEllipse>1){longitudinal/=actualEllipse;lateral/=actualEllipse;sliding=true;}
        // Solved load remains instantaneous, bounded, and dissipative after velocity reversal.
        double fx=admissible(longitudinal*capX,slip,dt,inverseLongMass);
        double fy=-admissible(-lateral*capY,lateralSpeed,dt,inverseLatMass);
        return new TireFriction.Force(fx,fy,sliding);
    }
    private static double admissible(double force,double slip,double dt,double inverseMass) {
        if(force*slip<=0)return 0;
        return Math.copySign(Math.min(Math.abs(force),inverseMass==0?Double.POSITIVE_INFINITY:Math.abs(slip)/(dt*inverseMass)),force);
    }
}
