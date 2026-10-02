package simrunner;

import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import static org.junit.jupiter.api.Assertions.*;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.system.NativeLibraryLoader;
import java.nio.file.*;
import java.util.*;
import java.util.zip.*;

class RobotMotionDemoTest {
    @TempDir Path tmp;
    @BeforeAll static void nativePhysics(){NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    String fixture(boolean differential,boolean mechanisms) {
        String xml="<robot name=\"Motion fixture\"><link name=\"base\"><visual><geometry><box size=\".3 .2 .1\"/></geometry></visual></link>";
        List<String> names=differential?List.of("leftDrive","rightDrive"):DriveGeometry.MOTORS;
        for(int i=0;i<names.size();i++) {
            xml+="<link name=\"wheel"+i+"\"><visual><geometry><cylinder radius=\".045\" length=\".02\"/></geometry></visual></link>"
                +"<joint name=\"wheel_joint"+i+"\" type=\"continuous\"><parent link=\"base\"/><child link=\"wheel"+i+"\"/><origin xyz=\""+(i<2?.1:-.1)+" "+(i%2==0?.13:-.13)+" 0\"/><axis xyz=\"0 1 0\"/></joint>"
                +"<transmission name=\"wheel_tx"+i+"\"><joint name=\"wheel_joint"+i+"\"/><actuator name=\""+names.get(i)+"\"><mechanicalReduction>1</mechanicalReduction></actuator></transmission>";
        }
        if(mechanisms) {
            xml+=joint("arm","revolute","0 0 1","-.3",".3","armMotor", "0 0 .3");
            xml+=joint("slide","prismatic","1 0 0","0",".08","slideMotor", ".3 0 .3");
            xml+=joint("gate","revolute","0 0 1","0",".5","gateServo", "-.3 0 .3");
            xml+=joint("passive","revolute","0 0 1","-.2",".2",null, "0 .4 .3");
        }
        return xml+"</robot>";
    }
    String joint(String name,String type,String axis,String low,String high,String motor,String xyz) {
        return "<link name=\""+name+"\"><visual><geometry><box size=\".06 .03 .03\"/></geometry></visual></link>"
            +"<joint name=\""+name+"_joint\" type=\""+type+"\"><parent link=\"base\"/><child link=\""+name+"\"/><origin xyz=\""+xyz+"\"/><axis xyz=\""+axis+"\"/><limit lower=\""+low+"\" upper=\""+high+"\" effort=\"5\" velocity=\"2\"/></joint>"
            +(motor==null?"":"<transmission name=\""+name+"_tx\"><joint name=\""+name+"_joint\"/><actuator name=\""+motor+"\"><mechanicalReduction>1</mechanicalReduction></actuator></transmission>");
    }
    GuidedSetupController model(boolean differential,boolean mechanisms)throws Exception {
        var controller=new GuidedSetupController(tmp.resolve("library"),null);controller.session.start("robot",null);
        Path zip=tmp.resolve("robot.zip");try(var out=new ZipOutputStream(Files.newOutputStream(zip))){out.putNextEntry(new ZipEntry("model.urdf"));out.write(fixture(differential,mechanisms).getBytes(java.nio.charset.StandardCharsets.UTF_8));out.closeEntry();}
        controller.load("robot","cad",zip,null);
        var p=controller.session.profile("robot");var runtime=FieldPackage.map(p.get("runtime"));
        if(differential)runtime.put("drive",Map.of("type","differential","left_motor","leftDrive","right_motor","rightDrive","wheel_radius_m",.045,"track_width_m",.26,"left_shaft_sign",-1,"right_shaft_sign",1));
        if(mechanisms)runtime.put("servoPhysics",Map.of("gateServo",Map.of("stall_torque_nm",1.,"no_load_speed_rad_s",3.,"travel_rad",.5,"position_gain_per_s",8.,"velocity_gain_nm_per_rad_s",.2,"deadband_rad",.001)));
        FieldPackage.map(p.get("parameters")).put("friction",0.);controller.update("robot",p);controller.compile("robot");return controller;
    }
    void complete(RobotMotionDemo demo) {
        for(int i=0;i<30000&&!demo.finished();i++)demo.tick();assertTrue(demo.finished());assertEquals("",demo.failure);
    }
    @ParameterizedTest @ValueSource(booleans={true,false})
    void nativeDriveDemoMovesOnlyInItsConfiguredDirectionsAndLeavesDraftUnreviewed(boolean differential)throws Exception {
        var c=model(differential,false);Path file=c.session.modelPath("robot");String original=Files.readString(file);var setup=c.motionSetup();
        assertEquals(differential?4:6,setup.plan().drive.size());assertEquals(!differential,setup.plan().drive.stream().anyMatch(m->m.left()!=0));
        assertTrue(setup.hardwareLabel().contains("Generic demo"));
        try(var demo=new RobotMotionDemo(setup,new DesktopAssetManager(true))) {
            complete(demo);assertEquals(differential?4:6,demo.observations.size());
            for(var result:demo.observations)assertEquals("movement observed",result.get("outcome"),result.toString());
            assertTrue((Boolean)demo.report().get("complete"));
            demo.stop();assertTrue((Boolean)demo.report().get("complete"));assertEquals("finished",demo.report().get("status"));
            assertEquals(demo.actions.size(),demo.completedMovements());
            for(var motor:setup.hardware().getAll(simcore.SimDcMotorEx.class))assertEquals(0,motor.getPower());
        }
        assertEquals(original,Files.readString(file));assertFalse(c.session.ready("robot"));assertFalse(Files.exists(file.getParent().resolve("validation.json")));
        var report=new LinkedHashMap<String,Object>();report.put("context",setup.context());report.put("status","finished");report.put("hardware",setup.hardwareLabel());report.put("observations",List.of());ProfileIO.save(c.motionReport(),report);
        assertTrue(c.motionResults().contains("finished"));c.measurements(new RobotMeasurements.Entry(8.,null,null,null));assertTrue(c.motionResults().contains("changed"));
    }
    @Test void nativeHingeSlideAndServoDemonstrateTravelWithinLimits()throws Exception {
        var c=model(true,true);var setup=c.motionSetup();assertEquals(3,setup.plan().mechanisms.size());assertTrue(setup.plan().notes.stream().anyMatch(s->s.contains("passive_joint")&&s.contains("unbound/passive")));
        try(var demo=new RobotMotionDemo(setup,new DesktopAssetManager(true))) {
            for(var action:demo.actions)if(action.mechanism()!=null){assertTrue(action.target()>=action.mechanism().joint().lower());assertTrue(action.target()<=action.mechanism().joint().upper());}
            complete(demo);assertEquals(10,demo.observations.size());
            for(var row:demo.observations)assertEquals("movement observed",row.get("outcome"),row.toString());
            assertTrue(demo.observations.stream().anyMatch(row->"m".equals(row.get("unit"))&&FieldPackage.num(row,"peak_travel")>.005));
            for(var m:setup.plan().mechanisms){double q=demo.robot.jointPosition(m.joint().name());assertTrue(q>=m.joint().lower()-.01&&q<=m.joint().upper()+.01,m.joint().name()+" "+q);}
        }
    }
    @Test void pauseAndStopZeroMotorCommandsAndFreezePhysics()throws Exception {
        var c=model(true,false);var setup=c.motionSetup();try(var demo=new RobotMotionDemo(setup,new DesktopAssetManager(true))) {
            for(int i=0;i<400;i++)demo.tick();assertTrue(setup.hardware().getAll(simcore.SimDcMotorEx.class).stream().anyMatch(m->m.getPower()!=0));
            demo.pause();var before=demo.world.getChassisPosition();for(int i=0;i<480;i++)demo.tick();assertEquals(before,demo.world.getChassisPosition());assertTrue(setup.hardware().getAll(simcore.SimDcMotorEx.class).stream().allMatch(m->m.getPower()==0));
            demo.pause();for(int i=0;i<20;i++)demo.tick();demo.stop();assertEquals("stopped",demo.report().get("status"));assertFalse((Boolean)demo.report().get("complete"));assertTrue(setup.hardware().getAll(simcore.SimDcMotorEx.class).stream().allMatch(m->m.getPower()==0));
        }
    }
    @Test void missingDriveBindingsDoNotInventMecanumMotionAndProjectHardwareIsStrict()throws Exception {
        var c=model(false,false);var p=c.session.profile("robot");var part=FieldPackage.map(FieldPackage.map(FieldPackage.map(p.get("entities")).get("wheel0")).get("settings"));part.put("actuators",List.of());c.update("robot",p);c.compile("robot");var setup=c.motionSetup();assertTrue(setup.plan().drive.isEmpty());assertTrue(setup.plan().notes.stream().anyMatch(s->s.contains("left_front_drive")));
        Path project=tmp.resolve("team");Files.createDirectories(project);Files.writeString(project.resolve("robot.xml"),"<Robot><LynxUsbDevice name=\"hub\"><LynxModule name=\"module\"><Motor name=\"unrelated\" port=\"0\"/></LynxModule></LynxUsbDevice></Robot>");ProfileIO.save(project.resolve("motors.json"),Map.of("motors",Map.of()));ProfileIO.save(project.resolve("sim.config"),Map.of("robotConfig","robot.xml","presetMotors","motors.json"));String original=Files.readString(project.resolve("sim.config"));
        assertThrows(IllegalArgumentException.class,()->RobotMotionDemo.setup(c.session.modelPath("robot"),project));assertEquals(original,Files.readString(project.resolve("sim.config")));
    }
    @Test void unboundMechanismsAndMotionPageDoNotBypassMigrationReview()throws Exception {
        var c=model(true,true);var p=c.session.profile("robot");var arm=FieldPackage.map(FieldPackage.map(FieldPackage.map(p.get("entities")).get("arm")).get("settings"));arm.put("actuators",List.of());c.update("robot",p);c.compile("robot");var setup=c.motionSetup();assertEquals(2,setup.plan().mechanisms.size());
        var motion=new GuidedSetupSession.Step("robot","motion");assertTrue(c.session.steps().contains(motion));assertFalse(c.session.complete(motion));c.continueAfterMotion();assertTrue(c.session.complete(motion));assertFalse(c.session.complete(new GuidedSetupSession.Step("robot","review")));assertFalse(Files.exists(c.session.modelPath("robot").getParent().resolve("validation.json")));
        c.session.start("field",null);assertFalse(c.session.steps().stream().anyMatch(s->s.page().equals("motion")));
        c.session.start("robot",null);FieldPackage.map(p.get("migration")).put("pending",List.of(Map.of("reason","ambiguous")));c.update("robot",p);c.compile("robot");assertThrows(IllegalArgumentException.class,c::motionSetup);
    }
    @Test void coupledContinuousFollowerMovesWithItsSourceWithoutAnIndependentDemo()throws Exception {
        var c=model(true,false);
        String xml=fixture(true,false).replace("</robot>",joint("intake","continuous","0 0 1","-1","1","intakeMotor","0 0 .3")
            +joint("follower","continuous","0 0 1","-1","1",null,".1 0 .3").replace("<limit lower=\"-1\" upper=\"1\" effort=\"5\" velocity=\"2\"/>","<mimic joint=\"intake_joint\" multiplier=\"1\"/>")+"</robot>");
        Path zip=tmp.resolve("coupled.zip");try(var out=new ZipOutputStream(Files.newOutputStream(zip))){out.putNextEntry(new ZipEntry("model.urdf"));out.write(xml.getBytes(java.nio.charset.StandardCharsets.UTF_8));out.closeEntry();}
        var previousRuntime=c.session.profile("robot").get("runtime");c.load("robot","cad",zip,null);var p=c.session.profile("robot");p.put("runtime",previousRuntime);FieldPackage.map(p.get("parameters")).put("friction",0.);c.update("robot",p);c.compile("robot");var setup=c.motionSetup();
        assertEquals(1,setup.plan().mechanisms.size());assertTrue(setup.plan().notes.stream().anyMatch(s->s.contains("follower_joint")&&s.contains("intake_joint")));
        try(var demo=new RobotMotionDemo(setup,new DesktopAssetManager(true))) {complete(demo);assertTrue(Math.abs(demo.robot.jointPosition("intake_joint"))>.05);assertEquals(demo.robot.jointPosition("intake_joint"),demo.robot.jointPosition("follower_joint"),.03);assertTrue(demo.actions.stream().noneMatch(a->a.label().startsWith("follower")));}
    }
    @Test void stalledMotorsAreReportedAsNoMotionRatherThanSuccessfulAnimation()throws Exception {
        var c=model(true,false);var setup=c.motionSetup();for(var motor:setup.hardware().getAll(simcore.SimDcMotorEx.class)){motor.setTargetPosition(0);motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);}
        try(var demo=new RobotMotionDemo(setup,new DesktopAssetManager(true))) {complete(demo);for(var row:demo.observations)assertEquals("no clear movement",row.get("outcome"),row.toString());}
    }
}
