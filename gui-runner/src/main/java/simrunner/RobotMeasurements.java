package simrunner;

import java.util.*;
import simcore.MiniJson;

/** Manual measurement entry is separate from recording-derived response calibration. */
final class RobotMeasurements {
    enum MassUnit {
        KG("kg",1), LB("lb",.45359237);
        final String label; final double kg;
        MassUnit(String label,double kg){this.label=label;this.kg=kg;}
        public String toString(){return label;}
    }
    enum LengthUnit {
        MM("mm",.001), CM("cm",.01), IN("in",.0254);
        final String label; final double meters;
        LengthUnit(String label,double meters){this.label=label;this.meters=meters;}
        public String toString(){return label;}
    }
    record Entry(Double massKg, Double diameterM, Double trackM, Double wheelbaseM) {
        Entry {
            for (Double value : new Double[]{massKg,diameterM,trackM,wheelbaseM})
                if (value!=null && (!Double.isFinite(value)||value<=0))
                    throw new IllegalArgumentException("Enter positive, finite measurements for every selected section.");
        }
        boolean empty(){return massKg==null && diameterM==null && trackM==null && wheelbaseM==null;}
    }
    static Map<String,Object> apply(Map<String,Object> original, Entry entry) {
        if (!"robot".equals(original.get("model_kind"))) throw new IllegalArgumentException("Measurements apply to a robot model.");
        var profile=MiniJson.parseObject(ProfileIO.json(original));
        var runtime=FieldPackage.map(profile.get("runtime"));
        var provenance=FieldPackage.map(profile.get("provenance"));
        if (entry.massKg!=null) measured(runtime,provenance,"total_mass_kg",entry.massKg,"operating robot with battery");
        if (runtime.containsKey("drive")) {
            var drive=FieldPackage.map(runtime.get("drive"));
            if (entry.diameterM!=null) measured(drive,provenance,"wheel_radius_m",entry.diameterM/2,"tread diameter", "runtime/drive/");
            if (entry.trackM!=null) measured(drive,provenance,"track_width_m",entry.trackM,"left/right wheel center spacing", "runtime/drive/");
            if (entry.wheelbaseM!=null) throw new IllegalArgumentException("Differential drive uses track width; wheelbase is not a runtime input.");
            DifferentialDriveConfig.parse(drive);
        } else if (entry.diameterM!=null || entry.trackM!=null || entry.wheelbaseM!=null) {
            if (entry.diameterM==null || entry.trackM==null || entry.wheelbaseM==null)
                throw new IllegalArgumentException("Mecanum measurements need diameter, track width and wheelbase together.");
            var geometry=new LinkedHashMap<String,Object>();
            measured(geometry,provenance,"wheel_radius_m",entry.diameterM/2,"common tread diameter", "runtime/drive_geometry/");
            measured(geometry,provenance,"track_width_m",entry.trackM,"left/right wheel center spacing", "runtime/drive_geometry/");
            measured(geometry,provenance,"wheelbase_m",entry.wheelbaseM,"front/rear axle center spacing", "runtime/drive_geometry/");
            DriveGeometry.parse(geometry);runtime.put("drive_geometry",geometry);
        }
        return profile;
    }
    private static void measured(Map<String,Object> target,Map<String,Object> provenance,String key,double value,String method) {
        measured(target,provenance,key,value,method,"runtime/");
    }
    private static void measured(Map<String,Object> target,Map<String,Object> provenance,String key,double value,String method,String path) {
        target.put(key,value);provenance.put(path+key,"measured manually: "+method);
    }
}
