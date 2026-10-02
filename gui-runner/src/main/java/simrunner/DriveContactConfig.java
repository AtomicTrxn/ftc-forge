package simrunner;

import java.util.Map;

/** Explicit ownership of flat static wheel support; absent/disabled retains legacy physics. */
record DriveContactConfig(boolean enabled,double minSupportNormalY,double maxContactGapM,double rollingResistanceCoefficient) {
    DriveContactConfig {
        if(!Double.isFinite(minSupportNormalY)||minSupportNormalY<.5||minSupportNormalY>1
            ||!Double.isFinite(maxContactGapM)||maxContactGapM<0||maxContactGapM>.02
            ||!Double.isFinite(rollingResistanceCoefficient)||rollingResistanceCoefficient<0||rollingResistanceCoefficient>1)
            throw new IllegalArgumentException("Invalid drive contact normal, gap or rolling resistance. Review wheel/floor geometry.");
    }
    static Map<String,Object> defaults(boolean enabled) {
        return new java.util.LinkedHashMap<>(Map.of("enabled",enabled,"min_support_normal_y",.7,"max_contact_gap_m",.003,"rolling_resistance_coefficient",.005));
    }
    static DriveContactConfig parse(Map<String,Object> values) {
        if(!(values.get("enabled") instanceof Boolean enabled))throw new IllegalArgumentException("drive_contacts.enabled must be true or false");
        return new DriveContactConfig(enabled,FieldPackage.num(values,"min_support_normal_y"),FieldPackage.num(values,"max_contact_gap_m"),FieldPackage.num(values,"rolling_resistance_coefficient"));
    }
}
