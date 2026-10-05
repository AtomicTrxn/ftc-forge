package simrunner;

import physics.TireFriction;
import physics.TireResponse;
import java.util.List;
import java.util.Map;

public record TireDriveConfig(TireFriction.Spec traction, TireFriction.Spec omni,
                              List<String> omniJoints, double reflectedMotorInertiaKgM2, double contactToleranceM,
                              TireResponse.Spec tractionResponse,TireResponse.Spec omniResponse) {
    public TireDriveConfig(TireFriction.Spec traction,TireFriction.Spec omni,List<String> omniJoints,double inertia,double tolerance) {
        this(traction,omni,omniJoints,inertia,tolerance,TireResponse.Spec.steady(traction),TireResponse.Spec.steady(omni));
    }
    public TireDriveConfig {
        if(traction==null||omni==null||omniJoints==null||tractionResponse==null||omniResponse==null)throw new IllegalArgumentException("Missing tire specifications");
        omniJoints=List.copyOf(omniJoints);
        if(omniJoints.stream().anyMatch(String::isBlank)||omniJoints.stream().distinct().count()!=omniJoints.size())
            throw new IllegalArgumentException("omni_joints needs unique nonblank joint names");
        if(!Double.isFinite(reflectedMotorInertiaKgM2)||reflectedMotorInertiaKgM2<=0
            ||!Double.isFinite(contactToleranceM)||contactToleranceM<=0||contactToleranceM>.02)
            throw new IllegalArgumentException("Invalid tire inertia/contact tolerance");
    }
    @SuppressWarnings("unchecked") static TireDriveConfig parse(Map<String,Object> values) {
        return parse(values,true);
    }
    @SuppressWarnings("unchecked") static TireDriveConfig parse(Map<String,Object> values,boolean checkMeasurements) {
        if(!(values.getOrDefault("omni_joints",List.of()) instanceof List<?> names)||names.stream().anyMatch(x->!(x instanceof String)))
            throw new IllegalArgumentException("omni_joints requires an array of joint names");
        var traction=spec((Map<String,Object>)values.get("traction"));var omni=spec((Map<String,Object>)values.get("omni"));
        if(checkMeasurements)for(String group:List.of("traction","omni")){var v=FieldPackage.map(values.get(group));if(v.containsKey("measurement"))try{TireSlipMeasurement.check(FieldPackage.map(v.get("measurement")),group,group.equals("traction")?traction:omni);}catch(Exception e){throw new IllegalArgumentException("Tire class "+group+": "+e.getMessage(),e);}}
        return new TireDriveConfig(traction,omni,
            (List<String>)values.getOrDefault("omni_joints",List.of()),n(values,"reflected_motor_inertia_kg_m2"),n(values,"contact_tolerance_m"),
            response(values,"traction_response",traction),response(values,"omni_response",omni));
    }
    private static TireResponse.Spec response(Map<String,Object> values,String key,TireFriction.Spec tire) {
        if(!values.containsKey(key))return TireResponse.Spec.steady(tire);
        var v=FieldPackage.map(values.get(key));
        if(!v.keySet().equals(java.util.Set.of("lateral_stiffness_n_per_mps","lateral_mu","relaxation_time_s")))throw new IllegalArgumentException(key+" requires lateral_stiffness_n_per_mps, lateral_mu, relaxation_time_s");
        return new TireResponse.Spec(SceneSensorConfig.n(v,"lateral_stiffness_n_per_mps",0,0,100000),SceneSensorConfig.n(v,"lateral_mu",0,0,2),SceneSensorConfig.n(v,"relaxation_time_s",0,0,2));
    }
    private static TireFriction.Spec spec(Map<String,Object> v) {
        if(v==null)throw new IllegalArgumentException("tires requires traction and omni specs");
        return new TireFriction.Spec(n(v,"static_mu"),n(v,"sliding_mu"),n(v,"lateral_scale"),n(v,"stiffness_n_per_mps"),n(v,"transition_mps"));
    }
    private static double n(Map<String,Object> v,String k){return DifferentialDriveConfig.number(v,k);}
}
