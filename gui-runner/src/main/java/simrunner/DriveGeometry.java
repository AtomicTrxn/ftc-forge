package simrunner;

import java.util.*;
import simcore.RobotUrdf;

/** Optional measured Mecanum dimensions; absence retains CAD inference and legacy defaults. */
record DriveGeometry(double wheelRadiusM, double trackWidthM, double wheelbaseM) {
    static final List<String> MOTORS = List.of("left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive");
    DriveGeometry {
        for (double value : new double[]{wheelRadiusM, trackWidthM, wheelbaseM})
            if (!Double.isFinite(value) || value <= 0)
                throw new IllegalArgumentException("Drive dimensions must be positive and finite.");
    }
    static DriveGeometry parse(Map<String,Object> values) {
        return new DriveGeometry(DifferentialDriveConfig.number(values,"wheel_radius_m"),
            DifferentialDriveConfig.number(values,"track_width_m"), DifferentialDriveConfig.number(values,"wheelbase_m"));
    }
    record Resolved(double trackWidthM, double wheelbaseM, List<Double> wheelRadii, boolean positionsFromCad) { }
    static Resolved resolve(RobotUrdf urdf, DriveGeometry measured) {
        if (measured != null) return new Resolved(measured.trackWidthM, measured.wheelbaseM,
            Collections.nCopies(4, measured.wheelRadiusM), false);
        double track=.30, wheelbase=.35;
        var radii=new ArrayList<>(Collections.nCopies(4,.048));
        double[] x=new double[4], y=new double[4]; boolean[] found=new boolean[4];
        if (urdf != null) for (var tx : urdf.transmissions.values()) {
            var joint=urdf.joints.get(tx.joint());
            if (!joint.type().equals("continuous")) continue;
            for (var actuator : tx.actuators()) {
                int i=MOTORS.indexOf(actuator.name()); if (i<0) continue;
                if (joint.parent().equals(urdf.rootLink)) {x[i]=joint.origin().xyz()[0];y[i]=joint.origin().xyz()[1];found[i]=true;}
                for (var collision : urdf.links.get(joint.child()).collisions())
                    if (collision.geometry().kind().equals("cylinder")) radii.set(i,collision.geometry().dimensions()[0]);
            }
        }
        boolean cad=found[0]&&found[1]&&found[2]&&found[3];
        if (cad) {
            track=(Math.abs(y[0]-y[1])+Math.abs(y[2]-y[3]))/2;
            wheelbase=(Math.abs(x[0]-x[2])+Math.abs(x[1]-x[3]))/2;
            if (track<=0 || wheelbase<=0) throw new IllegalArgumentException("Imported drive-wheel layout has zero track width or wheelbase");
        }
        return new Resolved(track,wheelbase,List.copyOf(radii),cad);
    }
}
