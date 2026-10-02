package simrunner;

import java.util.*;

/** Explicit ownership of flat static wheel support; absent/disabled retains legacy physics. */
record DriveContactConfig(boolean enabled,double minSupportNormalY,double maxContactGapM,double rollingResistanceCoefficient,Map<String,Double> wheelFriction) {
    DriveContactConfig {
        if(!Double.isFinite(minSupportNormalY)||minSupportNormalY<.5||minSupportNormalY>1
            ||!Double.isFinite(maxContactGapM)||maxContactGapM<0||maxContactGapM>.02
            ||!Double.isFinite(rollingResistanceCoefficient)||rollingResistanceCoefficient<0||rollingResistanceCoefficient>1)
            throw new IllegalArgumentException("Invalid drive contact normal, gap or rolling resistance. Review wheel/floor geometry.");
        wheelFriction=Map.copyOf(wheelFriction);
        wheelFriction.forEach((joint,mu)->{if(joint.isBlank()||!Double.isFinite(mu)||mu<0||mu>2)throw new IllegalArgumentException("Wheel friction requires a drive joint and a finite coefficient from 0 to 2.");});
    }
    double friction(String joint,double inherited){return wheelFriction.getOrDefault(joint,inherited);}
    static Map<String,Object> defaults(boolean enabled) {
        return new java.util.LinkedHashMap<>(Map.of("enabled",enabled,"min_support_normal_y",.7,"max_contact_gap_m",.003,"rolling_resistance_coefficient",.005));
    }
    static DriveContactConfig parse(Map<String,Object> values) {
        if(!(values.get("enabled") instanceof Boolean enabled))throw new IllegalArgumentException("drive_contacts.enabled must be true or false");
        var grip=new LinkedHashMap<String,Double>();
        if(values.containsKey("wheel_friction")) {
            if(!(values.get("wheel_friction") instanceof List<?> entries))throw new IllegalArgumentException("wheel_friction must be a list of joint/friction settings.");
            for(Object entry:entries) {
                var setting=FieldPackage.map(entry);
                if(!(setting.get("joint") instanceof String joint)||!(setting.get("friction") instanceof Number mu))throw new IllegalArgumentException("Wheel friction requires a drive joint and numeric coefficient.");
                if(grip.putIfAbsent(joint,mu.doubleValue())!=null)throw new IllegalArgumentException("Duplicate wheel friction setting: "+joint);
            }
        }
        return new DriveContactConfig(enabled,FieldPackage.num(values,"min_support_normal_y"),FieldPackage.num(values,"max_contact_gap_m"),FieldPackage.num(values,"rolling_resistance_coefficient"),grip);
    }
}
