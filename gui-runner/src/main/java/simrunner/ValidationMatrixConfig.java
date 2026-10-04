package simrunner;

import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import simcore.MiniJson;

/** Versioned, bounded test ranges and explicit acceptance tolerances; not runtime model tuning. */
final class ValidationMatrixConfig {
    final Map<String,Object> values;
    private static final Map<String,double[]> RANGES=Map.of(
        "mass_kg",new double[]{.5,20},"wheel_radius_m",new double[]{.035,.12},"track_width_m",new double[]{.23,.8},
        "floor_friction",new double[]{0,1.2},"timestep_s",new double[]{1./480,1./30},"mechanism_reduction",new double[]{-20,20});
    private static final Map<String,double[]> TOLERANCES=Map.ofEntries(
        Map.entry("settle_s",new double[]{.4,2}),Map.entry("drive_s",new double[]{.5,2}),Map.entry("drift_tolerance_m",new double[]{.0001,.02}),
        Map.entry("mass_tolerance_kg",new double[]{.000001,.001}),Map.entry("impulse_tolerance_ns",new double[]{.0000001,.001}),
        Map.entry("velocity_bound_tolerance_mps",new double[]{.001,.05}),Map.entry("penetration_tolerance_m",new double[]{.001,.02}),
        Map.entry("convergence_absolute_m",new double[]{.001,.08}),Map.entry("convergence_relative",new double[]{.01,.25}),
        Map.entry("energy_increase_tolerance_j",new double[]{.00001,.01}),Map.entry("tire_reaction_tolerance_nm",new double[]{.0000001,.001}));
    ValidationMatrixConfig(Map<String,Object> values) {
        var keys=new HashSet<>(RANGES.keySet());keys.addAll(TOLERANCES.keySet());keys.add("schema_version");
        if(!values.keySet().equals(keys)||!(values.get("schema_version") instanceof Number n)||n.doubleValue()!=1)throw new IllegalArgumentException("Matrix requires exactly the schema version 1 fields");
        long cases=1;
        for(var e:RANGES.entrySet()) {
            if(!(values.get(e.getKey()) instanceof List<?> a)||a.isEmpty()||a.size()>4)throw new IllegalArgumentException("Matrix range needs 1..4 values: "+e.getKey());
            var unique=new HashSet<Double>();for(var v:a){double x=number(v,e.getKey());bounded(x,e.getValue(),e.getKey());if(e.getKey().equals("mechanism_reduction")&&Math.abs(x)<1)throw new IllegalArgumentException("Mechanism reduction magnitude must be 1..20");if(!unique.add(x))throw new IllegalArgumentException("Duplicate matrix value: "+e.getKey());}
            if(!e.getKey().equals("mechanism_reduction"))cases*=a.size();
        }
        if(cases>200)throw new IllegalArgumentException("Matrix budget is at most 200 drive configurations");
        if(list(values,"timestep_s").size()<2)throw new IllegalArgumentException("Convergence needs at least two timesteps");
        for(var e:TOLERANCES.entrySet())bounded(number(values.get(e.getKey()),e.getKey()),e.getValue(),e.getKey());
        this.values=MiniJson.parseObject(ProfileIO.json(values));
    }
    private static double number(Object value,String key){if(!(value instanceof Number n)||!Double.isFinite(n.doubleValue()))throw new IllegalArgumentException("Finite numeric matrix value required: "+key);return n.doubleValue();}
    private static void bounded(double v,double[] limits,String key){if(v<limits[0]-1e-12||v>limits[1]+1e-12)throw new IllegalArgumentException("Out of matrix range: "+key);}
    private static List<Double> list(Map<String,Object> v,String key){return ((List<?>)v.get(key)).stream().map(x->((Number)x).doubleValue()).toList();}
    List<Double> list(String key){return list(values,key);}
    double get(String key){return ((Number)values.get(key)).doubleValue();}
    static ValidationMatrixConfig load(Path path)throws Exception {
        if(path!=null)return new ValidationMatrixConfig(MiniJson.parseObject(Files.readString(path)));
        try(var in=ValidationMatrixConfig.class.getResourceAsStream("validation-matrix.json")){if(in==null)throw new IllegalStateException("Missing bundled validation matrix");return new ValidationMatrixConfig(MiniJson.parseObject(new String(in.readAllBytes(),StandardCharsets.UTF_8)));}
    }
}
