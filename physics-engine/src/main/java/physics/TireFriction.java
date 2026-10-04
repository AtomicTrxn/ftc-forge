package physics;

/** Dissipative brush contact, clipped by an anisotropic friction ellipse. SI units. */
public final class TireFriction {
    private TireFriction() {}
    public record Spec(double staticMu, double slidingMu, double lateralScale,
                       double stiffnessNPerMps, double transitionMps) {
        public Spec {
            for (double v : new double[]{staticMu, slidingMu, stiffnessNPerMps, transitionMps})
                if (!Double.isFinite(v) || v <= 0) throw new IllegalArgumentException("Tire constants must be finite and positive");
            if (slidingMu > staticMu || !Double.isFinite(lateralScale) || lateralScale < 0 || lateralScale > 1)
                throw new IllegalArgumentException("Tire sliding_mu <= static_mu; lateral_scale must be 0..1");
        }
    }
    public record Force(double longitudinalN, double lateralN, boolean sliding) {}
    public static double coefficient(Spec spec,double speedMps) {
        return spec.slidingMu+(spec.staticMu-spec.slidingMu)*Math.exp(-Math.pow(speedMps/spec.transitionMps,2));
    }
    /** Continuous steady, purely longitudinal envelope; no acceleration/inertia fit. */
    public static double steadyLongitudinal(Spec spec,double slipMps,double normalN) {
        if(!Double.isFinite(slipMps)||!Double.isFinite(normalN)||normalN<0)
            throw new IllegalArgumentException("Steady slip/load must be finite and load nonnegative");
        return Math.copySign(Math.min(spec.stiffnessNPerMps*Math.abs(slipMps),coefficient(spec,Math.abs(slipMps))*normalN),slipMps);
    }
    /** Slip is wheel surface speed minus hub speed; lateral velocity is hub relative to ground. */
    public static Force solve(Spec spec, double slipMps, double lateralMps, double normalN,
                              double dt, double inverseLongMass, double inverseLatMass) {
        for(double value:new double[]{slipMps,lateralMps,normalN,dt,inverseLongMass,inverseLatMass})
            if(!Double.isFinite(value))throw new IllegalArgumentException("Contact state must be finite");
        if(normalN<0||dt<=0||inverseLongMass<0||inverseLatMass<0)
            throw new IllegalArgumentException("Contact load/inverse mass must be nonnegative and timestep positive");
        if (normalN == 0) return new Force(0,0,false);
        double speed=Math.hypot(slipMps,lateralMps);
        double mu=coefficient(spec,speed);
        double cap=mu*normalN;
        double fx=spec.stiffnessNPerMps*slipMps/(1+spec.stiffnessNPerMps*dt*inverseLongMass);
        double ky=spec.stiffnessNPerMps*spec.lateralScale;
        double fy=-ky*lateralMps/(1+ky*dt*inverseLatMass);
        double ellipse=Math.hypot(fx/cap,spec.lateralScale==0 ? 0 : fy/(cap*spec.lateralScale));
        boolean sliding=ellipse>1;
        if(sliding){fx/=ellipse;fy/=ellipse;}
        return new Force(fx,fy,sliding);
    }
}
