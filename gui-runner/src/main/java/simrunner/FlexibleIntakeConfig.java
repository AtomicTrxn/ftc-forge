package simrunner;

import com.jme3.math.Vector3f;
import java.util.List;
import java.util.Map;

public record FlexibleIntakeConfig(List<String> links, int segmentsPerArm, double flexMassFraction,
    double widthM, double armLengthM, double thicknessM, double stiffnessNmPerRad,
    double dampingRatio, double maxBendRad, double friction, double contactStiffnessNPerM, double contactDampingNsPerM, Vector3f containmentMin, Vector3f containmentMax, com.jme3.math.Quaternion meshToBeam, Map<String,Integer> visualIndices) {
    public FlexibleIntakeConfig(List<String> links,int segmentsPerArm,double flexMassFraction,double widthM,double armLengthM,double thicknessM,double stiffnessNmPerRad,double dampingRatio,double maxBendRad,double friction,double contactStiffnessNPerM,double contactDampingNsPerM,Vector3f containmentMin,Vector3f containmentMax){this(links,segmentsPerArm,flexMassFraction,widthM,armLengthM,thicknessM,stiffnessNmPerRad,dampingRatio,maxBendRad,friction,contactStiffnessNPerM,contactDampingNsPerM,containmentMin,containmentMax,new com.jme3.math.Quaternion().fromAngleAxis(com.jme3.math.FastMath.HALF_PI,Vector3f.UNIT_Y),Map.of());}
    @Override public com.jme3.math.Quaternion meshToBeam(){return meshToBeam.clone();}
    public FlexibleIntakeConfig {
        if(links==null)throw new IllegalArgumentException("flexible_intake requires links");
        visualIndices=Map.copyOf(visualIndices);if(visualIndices.values().stream().anyMatch(i->i<0))throw new IllegalArgumentException("Flexible visual index must be nonnegative");
        meshToBeam=meshToBeam.clone();
        links=List.copyOf(links);
        if(links.isEmpty()||links.stream().anyMatch(n->n==null||n.isBlank())||links.stream().distinct().count()!=links.size()
            ||segmentsPerArm<2||segmentsPerArm>6||!Double.isFinite(flexMassFraction)||flexMassFraction<=0||flexMassFraction>=1)
            throw new IllegalArgumentException("Invalid flexible intake links/segments/mass allocation");
        for(double v:new double[]{widthM,armLengthM,thicknessM,stiffnessNmPerRad,dampingRatio,maxBendRad,friction,contactStiffnessNPerM,contactDampingNsPerM})
            if(!Double.isFinite(v)||v<=0)throw new IllegalArgumentException("Flexible intake constants must be positive and finite");
        if(maxBendRad>1.2||dampingRatio>1||thicknessM>=armLengthM/segmentsPerArm)
            throw new IllegalArgumentException("Flexible bend/damping/thickness out of bounds");
        containmentMin=containmentMin.clone();containmentMax=containmentMax.clone();
        for(int i=0;i<3;i++)if(!Float.isFinite(containmentMin.get(i))||!Float.isFinite(containmentMax.get(i))||containmentMin.get(i)>=containmentMax.get(i))
            throw new IllegalArgumentException("Invalid flexible intake containment bounds");
    }
    @Override public Vector3f containmentMin(){return containmentMin.clone();}
    @Override public Vector3f containmentMax(){return containmentMax.clone();}
    @SuppressWarnings("unchecked") static FlexibleIntakeConfig parse(Map<String,Object> v) {
        if(!(v.get("links") instanceof List<?> names)||names.stream().anyMatch(x->!(x instanceof String)))
            throw new IllegalArgumentException("flexible_intake.links requires an array of link names");
        double segments=n(v,"segments_per_arm");
        if(segments!=Math.rint(segments))throw new IllegalArgumentException("segments_per_arm must be an integer");
        Vector3f low=point(v,"containment_min_xyz_m"), high=point(v,"containment_max_xyz_m");
        // URDF y maps to negative jME z, reversing the bounds on that axis.
        float z=low.z;low.z=high.z;high.z=z;
        return new FlexibleIntakeConfig((List<String>)v.get("links"),(int)segments,n(v,"flex_mass_fraction"),n(v,"width_m"),
            n(v,"arm_length_m"),n(v,"thickness_m"),n(v,"stiffness_nm_per_rad"),n(v,"damping_ratio"),n(v,"max_bend_rad"),
            n(v,"friction"),n(v,"contact_stiffness_n_per_m"),n(v,"contact_damping_ns_per_m"),low,high,v.containsKey("mesh_to_beam_rpy_rad")?ImportedRobotScene.rotation(FieldPackage.vector(v.get("mesh_to_beam_rpy_rad"),3)):new com.jme3.math.Quaternion().fromAngleAxis(com.jme3.math.FastMath.HALF_PI,Vector3f.UNIT_Y),indices(v));
    }
    private static Map<String,Integer> indices(Map<String,Object> v){var out=new java.util.LinkedHashMap<String,Integer>();if(v.containsKey("visual_indices"))for(var e:FieldPackage.map(v.get("visual_indices")).entrySet()){if(!(e.getValue() instanceof Number n)||n.doubleValue()!=n.intValue())throw new IllegalArgumentException("Visual indices must be integers");out.put(e.getKey(),n.intValue());}return out;}
    private static double n(Map<String,Object> v,String k){return DifferentialDriveConfig.number(v,k);}
    private static Vector3f point(Map<String,Object> v,String k) {
        if(!(v.get(k) instanceof List<?> p)||p.size()!=3||p.stream().anyMatch(x->!(x instanceof Number)))
            throw new IllegalArgumentException(k+" requires three numeric coordinates");
        return ImportedRobotScene.position(p.stream().mapToDouble(x->((Number)x).doubleValue()).toArray());
    }
}
