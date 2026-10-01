package simrunner;

import com.jme3.math.Vector3f;
import java.util.List;
import java.util.Map;

/** Explicit, unmeasured compliant pinch approximation for the illustrative torus. */
public record TorusRetentionConfig(Vector3f seat, Vector3f exit, double contactDwellS,
        double travelMPerRad, double stiffnessNPerM, double dampingNsPerM,
        double maxForceN, double breakDistanceM, double reflectedShaftInertiaKgM2, double tiltRad,
        double angularStiffnessNmPerRad, double angularDampingNmsPerRad, double maxTorqueNm,
        double angularTravelRadPerRad, double contactStiffnessNPerM, double contactDampingNsPerM) {
    public TorusRetentionConfig {
        if (seat == null || exit == null) throw new IllegalArgumentException("Retention requires seat and exit");
        seat=seat.clone(); exit=exit.clone();
        for (Vector3f p : new Vector3f[]{seat,exit})
            for (int i=0;i<3;i++) if (!Float.isFinite(p.get(i))) throw new IllegalArgumentException("Nonfinite retention point");
        for (double v : new double[]{contactDwellS,travelMPerRad,stiffnessNPerM,dampingNsPerM,maxForceN,breakDistanceM,reflectedShaftInertiaKgM2,angularStiffnessNmPerRad,angularDampingNmsPerRad,maxTorqueNm,angularTravelRadPerRad,contactStiffnessNPerM,contactDampingNsPerM})
            if (!Double.isFinite(v) || !Float.isFinite((float)v) || (float)v<=0) throw new IllegalArgumentException("Retention constants must be positive and finite");
        if(!Double.isFinite(tiltRad)||Math.abs(tiltRad)>Math.PI/2) throw new IllegalArgumentException("Retention tilt must be finite and within 90 degrees");
        if(reflectedShaftInertiaKgM2>.1||maxTorqueNm>2||angularTravelRadPerRad>1) throw new IllegalArgumentException("Retention inertia/torque/travel outside supported bounds");
        if (exit.x <= seat.x || contactDwellS>.5 || travelMPerRad>.05 || maxForceN>20 || breakDistanceM>.3)
            throw new IllegalArgumentException("Retention geometry/force/travel outside supported bounds");
    }
    Vector3f seatNormal(){return new Vector3f((float)Math.sin(tiltRad),(float)Math.cos(tiltRad),0);}
    @Override public Vector3f seat(){return seat.clone();}
    @Override public Vector3f exit(){return exit.clone();}
    static TorusRetentionConfig parse(Map<String,Object> v) {
        return new TorusRetentionConfig(point(v,"seat_xyz_m"),point(v,"exit_xyz_m"),n(v,"contact_dwell_s"),
            n(v,"travel_m_per_rad"),n(v,"stiffness_n_per_m"),n(v,"damping_ns_per_m"),n(v,"max_force_n"),n(v,"break_distance_m"),n(v,"reflected_shaft_inertia_kg_m2"),n(v,"tilt_rad"),n(v,"angular_stiffness_nm_per_rad"),n(v,"angular_damping_nms_per_rad"),n(v,"max_torque_nm"),n(v,"angular_travel_rad_per_rad"),n(v,"contact_stiffness_n_per_m"),n(v,"contact_damping_ns_per_m"));
    }
    private static double n(Map<String,Object> v,String key){return DifferentialDriveConfig.number(v,key);}
    private static Vector3f point(Map<String,Object> v,String key) {
        if (!(v.get(key) instanceof List<?> xyz) || xyz.size()!=3 || xyz.stream().anyMatch(x->!(x instanceof Number)))
            throw new IllegalArgumentException(key+" requires three coordinates");
        return ImportedRobotScene.position(xyz.stream().mapToDouble(x->((Number)x).doubleValue()).toArray());
    }
}
