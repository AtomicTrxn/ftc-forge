package simrunner;
import java.nio.file.*;
import java.util.*;

/** Generated, non-calibrated fixture for reproducible native/rendered advanced feature checks. */
public final class AdvancedPhysicsFixture {
    public static void main(String[] args)throws Exception {
        if(args.length!=1)throw new IllegalArgumentException("AdvancedPhysicsFixture output-folder");
        Path folder=Path.of(args[0]).toAbsolutePath();Files.createDirectories(folder);
        var c=SyntheticRobots.rotatingRobot(folder.resolve("models"),true);var p=c.session.profile("robot");var runtime=FieldPackage.map(p.get("runtime"));runtime.put("start_height_m",.055);
        runtime.put("sensors",List.of(
            Map.of("name","range","type","distance","xyz_m",List.of(.2,0,.05),"max_range_m",2),
            Map.of("name","color","type","color","xyz_m",List.of(.2,0,.05),"max_range_m",2),
            Map.of("name","switch","type","touch","xyz_m",List.of(.15,0,0)),
            Map.of("name","camera","type","camera","xyz_m",List.of(.2,0,.15),"max_range_m",4)));
        c.update("robot",p);c.compile("robot");c.review("robot",true);
        Path project=folder.resolve("project");Files.createDirectories(project.resolve("TeamCode"));
        Files.writeString(project.resolve("TeamCode/AdvancedDemo.java"),"package fixture; import com.qualcomm.robotcore.eventloop.opmode.*; @TeleOp(name=\"Advanced fixture\") public class AdvancedDemo extends OpMode { public void init(){} public void loop(){} }");
        Files.writeString(project.resolve("robot_config.xml"),"<Robot><Webcam name='camera'/><LynxModule name='Control Hub'><Motor name='leftDrive' port='0'/><Motor name='rightDrive' port='1'/><IMU name='imu'/><DistanceSensor name='range'/><ColorSensor name='color'/><TouchSensor name='switch'/></LynxModule></Robot>");
        ProfileIO.save(project.resolve("preset_motors.json"),Map.of("name","Synthetic generic motors, not measured","motors",Map.of("leftDrive",motor(),"rightDrive",motor())));
        var behavior=Map.of("schema_version",1,"tags",List.of(Map.of("id",7,"name","Synthetic target","size_m",.16,"xyz_m",List.of(1,.2,.35),"rpy_rad",List.of(0,0,Math.PI))),"colors",List.of(Map.of("min_xyz_m",List.of(1.7,-2,0),"max_xyz_m",List.of(2,2,1),"rgba",List.of(220,10,20,255))),"rules",List.of(Map.of("id","practice-garden","alliance","neutral","mode","occupancy","types",List.of("pollen","red_nectar","blue_nectar"),"min_xyz_m",List.of(.6,-1.2,0),"max_xyz_m",List.of(1.6,1.2,.5),"points",1)));
        ProfileIO.save(project.resolve("sim.config"),Map.of("sourceRoot","TeamCode","robotConfig","robot_config.xml","presetMotors","preset_motors.json","robot_model_profile",c.session.modelPath("robot").toString(),"field",Map.of("source","generic","mode","game-pieces","piece_set","biobuzz"),"field_behavior",behavior));
        System.out.println("Advanced fixture: "+project+" | generated CAD and motor assumptions; no hardware calibration");
    }
    private static Map<String,Object> motor(){return Map.of("sku","generic synthetic","ratio",1,"tauStallNm",2,"iStallAmps",9.2,"omegaNoLoadRadS",30,"vNominal",12,"encoderCountsPerRev",500);}
}
