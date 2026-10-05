package simrunner;
import com.jme3.math.*;
import java.util.*;
/** Bounded poses and measurement assumptions, expressed in SI and URDF coordinates. */
record SceneSensorConfig(String name,String type,String link,Transform pose,double minRange,double maxRange,double updateHz,
                         double latencyS,double noiseM,long seed,double radiusM,double thresholdN,double horizontalFov,double verticalFov,double pressCos) {
    static List<SceneSensorConfig> parse(Object input) {
        var entries=FieldPackage.maps(input);if(entries.size()>32)throw new IllegalArgumentException("At most 32 scene sensors");
        var result=new ArrayList<SceneSensorConfig>();var names=new HashSet<String>();
        for(var m:entries) {
            if(!Set.of("name","type","link","xyz_m","rpy_rad","min_range_m","max_range_m","update_hz","latency_ms","noise_std_m","seed","contact_radius_m","threshold_n","horizontal_fov_rad","vertical_fov_rad","min_press_normal_cos").containsAll(m.keySet()))throw new IllegalArgumentException("Unknown scene sensor setting");
            String name=FieldPackage.str(m,"name"),type=FieldPackage.str(m,"type");if(!names.add(name)||!Set.of("distance","color","touch","camera").contains(type))throw new IllegalArgumentException("Sensor names must be unique and type distance/color/touch/camera");
            String link=m.containsKey("link")?FieldPackage.str(m,"link"):"";
            double min=n(m,"min_range_m",.01,0,100),max=n(m,"max_range_m",2,.001,100);if(max<=min)throw new IllegalArgumentException("Sensor max range must exceed min range");
            double hz=n(m,"update_hz",30,1,120),latency=n(m,"latency_ms",0,0,1000)/1000,noise=n(m,"noise_std_m",0,0,.1);
            double radius=n(m,"contact_radius_m",.015,.0001,.5),threshold=n(m,"threshold_n",.01,0,100000);
            double hfov=n(m,"horizontal_fov_rad",Math.toRadians(60),.01,Math.PI-.01),vfov=n(m,"vertical_fov_rad",Math.toRadians(45),.01,Math.PI-.01);
            double seed=n(m,"seed",0,-1e9,1e9);if(seed!=(long)seed)throw new IllegalArgumentException("Sensor seed must be an integer");
            result.add(new SceneSensorConfig(name,type,link,new Transform(FieldPackage.pos(m.getOrDefault("xyz_m",List.of(0,0,0))),ImportedRobotScene.rotation(FieldPackage.vector(m.getOrDefault("rpy_rad",List.of(0,0,0)),3))),min,max,hz,latency,noise,(long)seed,radius,threshold,hfov,vfov,n(m,"min_press_normal_cos",.5,0,1)));
        }
        return List.copyOf(result);
    }
    static double n(Map<String,Object> m,String key,double fallback,double low,double high){double value=m.containsKey(key)?FieldPackage.num(m,key):fallback;if(value<low||value>high)throw new IllegalArgumentException(key+" must be "+low+".."+high);return value;}
}
