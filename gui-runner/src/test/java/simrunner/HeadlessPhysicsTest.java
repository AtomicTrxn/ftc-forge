package simrunner;

import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import simcore.MiniJson;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.TimeUnit;
import static org.junit.jupiter.api.Assertions.*;

class HeadlessPhysicsTest {
    @TempDir Path tmp;
    @BeforeAll static void nativeLibrary(){com.jme3.system.NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    private static final List<String> DRIVES = DriveGeometry.MOTORS;
    private static final String IMPORTS = "package headlessfixture; import com.qualcomm.robotcore.eventloop.opmode.*; import com.qualcomm.robotcore.hardware.*; import org.firstinspires.ftc.robotcore.external.navigation.*; import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver; import com.qualcomm.hardware.sparkfun.SparkFunOTOS; ";
    private static final String MOTORS = "DcMotorEx[] motors=hardwareMap.getAll(DcMotorEx.class).toArray(DcMotorEx[]::new);";
    private Path project(String source, Map<String,Object> extra, List<String> names) throws Exception {
        Path p=tmp.resolve("project-"+UUID.randomUUID());Files.createDirectories(p.resolve("TeamCode"));
        Files.writeString(p.resolve("TeamCode/Probe.java"),IMPORTS+source);
        StringBuilder xml=new StringBuilder("<Robot><LynxModule name='Hub'>");
        for(int i=0;i<names.size();i++)xml.append("<Motor name='").append(names.get(i)).append("' port='").append(i).append("'/>");
        xml.append("<IMU name='imu'/><GoBildaPinpointDriver name='pinpoint'/><SparkFunOTOS name='otos'/><DistanceSensor name='range'/></LynxModule></Robot>");
        // Use canonical tags recognized by the SDK builder.
        Files.writeString(p.resolve("robot.xml"),xml.toString().replace("GoBildaPinpointDriver","Pinpoint").replace("SparkFunOTOS","OTOS"));
        var motors=new LinkedHashMap<String,Object>();
        for(String n:names)motors.put(n,Map.of("sku","synthetic","ratio",1,"tauStallNm",2,"iStallAmps",9.2,"omegaNoLoadRadS",30,"vNominal",12,"encoderCountsPerRev",500));
        ProfileIO.save(p.resolve("motors.json"),Map.of("name","Generated assumptions","motors",motors));
        var config=new LinkedHashMap<String,Object>(Map.of("sourceRoot","TeamCode","robotConfig","robot.xml","presetMotors","motors.json"));config.putAll(extra);
        ProfileIO.save(p.resolve("sim.config"),config);return p;
    }
    private Map<String,Object> run(Path p,double seconds)throws Exception {
        Path report=p.resolve("result.json");
        var r=HeadlessPhysicsRunner.run(p,"Probe",report,new HeadlessPhysicsRunner.Options(seconds,Math.round(seconds*1000)+2000,.25,1000));
        assertEquals(r.get("outcome"),MiniJson.parseObject(Files.readString(report)).get("outcome"));return r;
    }
    private static Map<String,Object> finalState(Map<String,Object> r){return FieldPackage.map(r.get("final"));}
    private static double x(Map<String,Object> r){return ((Number)((List<?>)finalState(r).get("xyz_m")).get(0)).doubleValue();}
    private static void passed(Map<String,Object> r){assertEquals(true,r.get("success"),r.toString());assertEquals(true,r.get("opmode_thread_terminated"));}

    @Test void actualClosedLoopTeamCodeReceivesNativePoseDistanceAndEncodersAndReloadsStaticState()throws Exception {
        String code="""
            @Autonomous public class Probe extends LinearOpMode {
              static int runs;
              public void runOpMode() throws InterruptedException {
                if(++runs!=1)throw new AssertionError("stale classloader");
                var p=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
                var o=hardwareMap.get(SparkFunOTOS.class,"otos");o.setLinearUnit(DistanceUnit.METER);
                var d=hardwareMap.get(DistanceSensor.class,"range");
                waitForStart();double initial=d.getDistance(DistanceUnit.METER);
                for(var m:hardwareMap.getAll(DcMotorEx.class))m.setPower(.5);
                while(opModeIsActive()){p.update();if(p.getPosX(DistanceUnit.METER)>.15)break;sleep(5);}
                if(!opModeIsActive())return;
                if(o.getPosition().x<.14)throw new AssertionError("OTOS not updated");
                if(d.getDistance(DistanceUnit.METER)>=initial-.08)throw new AssertionError("range not updated");
                for(var m:hardwareMap.getAll(DcMotorEx.class)){if(m.getCurrentPosition()<10)throw new AssertionError("encoder not updated");m.setPower(0);}
              }
            }
            """;
        var p=project(code,Map.of("robot_start_xyz_m",List.of(.25,.3,.1),"sensors",List.of(Map.of("name","range","type","distance","xyz_m",List.of(.23,0,0),"max_range_m",3))),DRIVES);
        for(int i=0;i<2;i++){var r=run(p,3);passed(r);assertEquals("completed",r.get("outcome"));assertTrue(x(r)>.39&&x(r)<.55);assertFalse((Boolean)r.get("renderer"));assertTrue(FieldPackage.maps(r.get("trajectory")).size()>2);}
    }
    @Test void imuClosedLoopTurnAndIterativeStopLifecycle()throws Exception {
        var p=project("""
            @TeleOp public class Probe extends OpMode {
              public void init(){hardwareMap.get(IMU.class,"imu").resetYaw();}
              public void start(){hardwareMap.get(DcMotorEx.class,"left_front_drive").setPower(-.3);hardwareMap.get(DcMotorEx.class,"left_back_drive").setPower(-.3);hardwareMap.get(DcMotorEx.class,"right_front_drive").setPower(.3);hardwareMap.get(DcMotorEx.class,"right_back_drive").setPower(.3);}
              public void loop(){if(hardwareMap.get(IMU.class,"imu").getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)>.3)requestOpModeStop();}
              public void stop(){for(var m:hardwareMap.getAll(DcMotorEx.class))m.setPower(0);}
            }
            """,Map.of(),DRIVES);
        var r=run(p,3);passed(r);assertEquals("completed",r.get("outcome"));assertTrue(FieldPackage.num(finalState(r),"yaw_rad")>.29);
    }
    @Test void nativeWallsBlockPoweredChassisAndDurationStopsTeleop()throws Exception {
        var p=project("@TeleOp public class Probe extends OpMode {public void init(){} public void start(){"+MOTORS+"for(var m:motors)m.setPower(.7);}public void loop(){}}",Map.of("robot_start_xyz_m",List.of(1.4,0,.1)),DRIVES);
        var r=run(p,1);passed(r);assertEquals("duration_complete",r.get("outcome"));assertTrue(x(r)>1.48&&x(r)<1.65);
        assertFalse(FieldPackage.maps(FieldPackage.map(r.get("diagnostics")).get("contacts")).isEmpty());
        assertEquals(true,r.get("motors_stopped"));
    }
    @Test void startupAndUserExceptionsProduceFailingEvidence()throws Exception {
        var p=project("@Autonomous public class Probe extends LinearOpMode {public void runOpMode() throws InterruptedException {waitForStart();throw new IllegalStateException(\"expected-probe-failure\");}}",Map.of(),DRIVES);
        var r=run(p,1);assertEquals(false,r.get("success"));assertEquals("opmode_failure",r.get("outcome"));assertTrue(r.get("failure").toString().contains("expected-probe-failure"));assertEquals(true,r.get("opmode_thread_terminated"));
        Files.writeString(p.resolve("sim.config"),"{}");r=run(p,1);assertEquals(false,r.get("success"));assertEquals("startup_failure",r.get("outcome"));assertTrue(Files.isRegularFile(p.resolve("result.json")));
    }
    @Test void cliWatchdogContainsInterruptIgnoringCodeInSeparateJvm()throws Exception {
        var p=project("@Autonomous public class Probe extends LinearOpMode {public void runOpMode() throws InterruptedException {waitForStart();for(;;){Thread.onSpinWait();}}}",Map.of(),DRIVES);
        Path report=p.resolve("watchdog.json"),log=p.resolve("cli.log");
        var process=new ProcessBuilder(Path.of(System.getProperty("java.home"),"bin/java").toString(),"-Djava.awt.headless=true","-cp",System.getProperty("java.class.path"),"simrunner.Main",p.toString(),"Probe","--physics","--watchdog-ms","150","--duration","2","--report",report.toString()).redirectErrorStream(true).redirectOutput(log.toFile()).start();
        try{assertTrue(process.waitFor(20,TimeUnit.SECONDS),"CLI failed to terminate");assertNotEquals(0,process.exitValue());var r=MiniJson.parseObject(Files.readString(report));assertEquals("watchdog",r.get("outcome"),Files.readString(log));assertEquals(false,r.get("opmode_thread_terminated"));assertEquals(false,r.get("success"));assertTrue(FieldPackage.num(FieldPackage.map(r.get("timing")),"simulation_s")<.25);}finally{process.destroyForcibly();}
    }
    @Test void reviewedRotatingWheelSuspensionSceneSensorsAndScoringWorkWithoutRenderer()throws Exception {
        Path fixture=tmp.resolve("advanced");AdvancedPhysicsFixture.main(new String[]{fixture.toString()});
        Path p=fixture.resolve("project");Files.writeString(p.resolve("TeamCode/Probe.java"),IMPORTS+"""
            @Autonomous public class Probe extends LinearOpMode {
              public void runOpMode()throws InterruptedException {
                waitForStart();if(hardwareMap.get(DistanceSensor.class,"range").getDistance(DistanceUnit.METER)>1.8)throw new AssertionError("no native range");
                hardwareMap.get(DcMotorEx.class,"leftDrive").setPower(-.2);hardwareMap.get(DcMotorEx.class,"rightDrive").setPower(.2);sleep(700);
              }
            }
            """);
        var r=run(p,2);passed(r);assertEquals("completed",r.get("outcome"));assertTrue(x(r)>.04, r.toString());
        assertEquals(4,FieldPackage.maps(FieldPackage.map(r.get("diagnostics")).get("wheels")).size());
        assertTrue(FieldPackage.num(FieldPackage.map(r.get("field_behavior")),"seconds")>.6);
        assertEquals(6,FieldPackage.maps(FieldPackage.map(r.get("field_behavior")).get("pieces")).size());
        assertEquals(1./480,FieldPackage.num(r,"fixed_step_s"),1e-9);
    }
    @Test void importedMechanismMotorEncoderFollowsNativeTravelLimit()throws Exception {
        var c=SyntheticRobots.importRobot(tmp.resolve("mechanisms"),true,true,false,SyntheticRobots.Dimensions.standard());c.review("robot",true);
        var p=project("""
            @Autonomous public class Probe extends LinearOpMode {
              public void runOpMode()throws InterruptedException {waitForStart();hardwareMap.get(DcMotorEx.class,"slideMotor").setPower(.2);sleep(500);}
            }
            """,Map.of("robot_model_profile",c.session.modelPath("robot").toString(),"start_height_m",.055),List.of("leftDrive","rightDrive","armMotor","slideMotor"));
        // URDF also binds the positional servo.
        String xml=Files.readString(p.resolve("robot.xml")).replace("</LynxModule>","<Servo name='gateServo'/></LynxModule>");Files.writeString(p.resolve("robot.xml"),xml);
        var r=run(p,2);passed(r);
        double slide=FieldPackage.num(FieldPackage.map(finalState(r).get("joints")),"slide_joint");assertEquals(.08,slide,.003);
        var motor=FieldPackage.map(FieldPackage.map(finalState(r).get("motors")).get("slideMotor"));
        assertTrue(FieldPackage.num(motor,"ticks")>0);assertEquals(.08,FieldPackage.num(motor,"shaft_rad"),.005);
        var joints=FieldPackage.maps(FieldPackage.map(r.get("diagnostics")).get("joints"));assertTrue(joints.stream().anyMatch(j->j.get("name").equals("slide_joint")&&j.get("position_unit").equals("m")));
    }
    @Test void reviewedImportedFieldModesKeepMetricBoundsAndSavedPiecePlacements()throws Exception {
        var editor=new ModelEditorController(tmp.resolve("field-library"));
        Path zip=SyntheticRobots.zip(tmp.resolve("field.zip"),"field.urdf",ModelPreparationTest.CAD);
        Path draft=Path.of(editor.run("import",zip.toString(),editor.library.toString(),"--kind","field").toString());
        var data=MiniJson.parseObject(Files.readString(draft));
        FieldPackage.map(FieldPackage.map(FieldPackage.map(data.get("entities")).get("ball")).get("settings")).put("role","piece");
        ProfileIO.save(draft,data);editor.run("compile",draft.toString());
        try(var nativeModel=new ModelValidation.Prepared(draft,new com.jme3.asset.DesktopAssetManager(true))){nativeModel.writeProof();}
        Path profile=Path.of(editor.run("save",draft.toString(),editor.library.toString(),"--reviewed").toString());
        for(String mode:List.of("field-only","game-pieces")) {
            var p=project("@TeleOp public class Probe extends OpMode {public void init(){}public void loop(){requestOpModeStop();}}",
                Map.of("field",Map.of("source","imported","package",profile.toString(),"mode",mode),"robot_start_xyz_m",List.of(-.4,.4,.12)),DRIVES);
            var r=run(p,1);passed(r);var scene=FieldPackage.map(r.get("scene"));
            assertEquals(List.of(1.,1.),scene.get("field_half_extents_m"));
            assertEquals(mode.equals("field-only")?0:1,((Number)scene.get("piece_count")).intValue());
            assertEquals(-.4,x(r),.01);
        }
    }
    @Test void importedFieldObstacleStallsMechanismAndItsEncoderTogether()throws Exception {
        var editor=new ModelEditorController(tmp.resolve("blocking-field"));
        String cad="<robot name='Slide blocker'><link name='floor'><visual><origin xyz='0 0 -.01'/><geometry><box size='2 2 .02'/></geometry></visual></link><link name='wall'><visual><geometry><box size='.01 .4 .8'/></geometry></visual></link><joint name='wall_mount' type='fixed'><parent link='floor'/><child link='wall'/><origin xyz='.345 0 .4'/></joint></robot>";
        Path zip=SyntheticRobots.zip(tmp.resolve("blocker.zip"),"field.urdf",cad);
        Path draft=Path.of(editor.run("import",zip.toString(),editor.library.toString(),"--kind","field").toString());
        try(var nativeModel=new ModelValidation.Prepared(draft,new com.jme3.asset.DesktopAssetManager(true))){nativeModel.writeProof();}
        Path field=Path.of(editor.run("save",draft.toString(),editor.library.toString(),"--reviewed").toString());
        var c=SyntheticRobots.importRobot(tmp.resolve("blocked-robot"),true,true,false,SyntheticRobots.Dimensions.standard());c.review("robot",true);
        var p=project("@Autonomous public class Probe extends LinearOpMode {public void runOpMode()throws InterruptedException {waitForStart();hardwareMap.get(DcMotorEx.class,\"slideMotor\").setPower(.2);sleep(500);}}",
            Map.of("robot_model_profile",c.session.modelPath("robot").toString(),"field",Map.of("source","imported","package",field.toString(),"mode","field-only"),"robot_start_xyz_m",List.of(0,0,.055)),List.of("leftDrive","rightDrive","armMotor","slideMotor"));
        Files.writeString(p.resolve("robot.xml"),Files.readString(p.resolve("robot.xml")).replace("</LynxModule>","<Servo name='gateServo'/></LynxModule>"));
        var r=run(p,2);passed(r);double q=FieldPackage.num(FieldPackage.map(finalState(r).get("joints")),"slide_joint");assertTrue(q<.02&&q>=0);
        var motor=FieldPackage.map(FieldPackage.map(finalState(r).get("motors")).get("slideMotor"));assertEquals(q,FieldPackage.num(motor,"shaft_rad"),.003);
        var diagnostics=FieldPackage.map(r.get("diagnostics"));assertFalse(FieldPackage.maps(diagnostics.get("contacts")).isEmpty());
        assertTrue(FieldPackage.maps(diagnostics.get("contacts")).stream().anyMatch(contact->contact.get("body").equals("slide")&&FieldPackage.num(contact,"normal_load_n")>.01));
        var slide=FieldPackage.maps(diagnostics.get("joints")).stream().filter(j->j.get("name").equals("slide_joint")).findFirst().orElseThrow();
        assertTrue(FieldPackage.num(slide,"effort")>.1);assertTrue(Math.abs(FieldPackage.num(slide,"velocity"))<.02);
    }
    @Test void cooperativeLinearWatchdogRequestsStopAndNoFurtherMotorClockExists()throws Exception {
        var p=project("@Autonomous public class Probe extends LinearOpMode {public void runOpMode()throws InterruptedException {waitForStart();"+MOTORS+"for(var m:motors)m.setPower(.3);while(opModeIsActive())sleep(5);}}",Map.of(),DRIVES);
        var r=HeadlessPhysicsRunner.run(p,"Probe",p.resolve("watchdog.json"),new HeadlessPhysicsRunner.Options(2,180,.25,1000));
        assertEquals("watchdog",r.get("outcome"));assertEquals(false,r.get("success"));assertEquals(true,r.get("opmode_thread_terminated"));assertEquals(true,r.get("motors_stopped"));
        assertTrue(Thread.getAllStackTraces().keySet().stream().noneMatch(t->t.getName().equals("motor-tick")&&t.isAlive()));
    }
    @Test void optionsRejectInvalidTimestepsAndDuration() {
        assertThrows(IllegalArgumentException.class,()->new HeadlessPhysicsRunner.Options(Double.NaN,1,0,1));
        assertThrows(IllegalArgumentException.class,()->new HeadlessPhysicsRunner.Options(1,0,0,1));
        assertThrows(IllegalArgumentException.class,()->new HeadlessPhysicsRunner.Options(1,1000,-1,1));
        assertThrows(IllegalArgumentException.class,()->simcore.HardwareMapBuilder.tickMotors(new com.qualcomm.robotcore.hardware.HardwareMap(),Double.NaN,0));
    }
}
