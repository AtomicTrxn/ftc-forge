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
import simcore.MiniJson;

class RobotMotionReviewTest {
    @TempDir Path tmp;
    @BeforeAll static void nativePhysics(){NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    RobotMotionDemoTest fixtures(){var f=new RobotMotionDemoTest();f.tmp=tmp;return f;}
    GuidedSetupController robot()throws Exception{return fixtures().model(true,true);}
    Map<String,Object> run(GuidedSetupController c,String group)throws Exception {
        try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),group)){fixtures().complete(demo);var result=demo.report();ProfileIO.save(c.motionReport(),result);return result;}
    }
    RobotMotionReview.Row row(RobotMotionReview.Snapshot snapshot,String id){return snapshot.rows().stream().filter(r->r.item().id().equals(id)).findFirst().orElseThrow();}
    void save(GuidedSetupController c,String id,String choice)throws Exception{c.saveMotionReviews(c.motionReview(),Map.of(id,choice),Map.of(id,"Compared against expected movement — café 🤖"));}
    void ready(GuidedSetupController c)throws Exception{try(var nativeModel=new ModelValidation.Prepared(c.session.modelPath("robot"),new DesktopAssetManager(true))){nativeModel.writeProof();}c.session.select("robot",Path.of(c.models.run("save",c.session.modelPath("robot").toString(),c.models.library.toString(),"--reviewed").toString()));}

    @Test void completedDemoCanBeReplacedButPreviousOrRunningReportCannot() {
        var finished=Map.<String,Object>of("run_id","new-run","complete",true,"status","finished");
        assertTrue(GuidedSetupController.completedNewMotionReport("old-run",finished));
        assertFalse(GuidedSetupController.completedNewMotionReport("new-run",finished));
        assertFalse(GuidedSetupController.completedNewMotionReport("old-run",Map.of("run_id","new-run","complete",false,"status","running")));
        assertFalse(GuidedSetupController.completedNewMotionReport("old-run",Map.of("run_id","new-run","complete",false,"status","stopped")));
        assertFalse(GuidedSetupController.completedNewMotionReport("old-run",Map.of()));
    }

    @Test void targetedDriveRunsOneDirectionAndNoMechanismCommands()throws Exception {
        var c=robot();var setup=c.motionSetup();try(var demo=new RobotMotionDemo(setup,new DesktopAssetManager(true),"drive/forward")) {
            assertEquals(1,demo.actions.size());for(int i=0;i<2000&&!demo.finished();i++){demo.tick();assertEquals(0,setup.hardware().get(simcore.SimDcMotorEx.class,"armMotor").getPower());assertEquals(0,setup.hardware().get(simcore.SimDcMotorEx.class,"slideMotor").getPower());}
            fixtures().complete(demo);assertEquals(1,demo.observations.size());assertEquals("drive/forward",demo.observations.get(0).get("id"));assertEquals("movement observed",demo.observations.get(0).get("outcome"));assertEquals("drive/forward",demo.report().get("selection"));
        }
        assertThrows(IllegalArgumentException.class,()->new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"gone"));
    }
    @ParameterizedTest @ValueSource(strings={"arm_joint","slide_joint","gate_joint"})
    void targetedMechanismRunsBothTargetsWithDriveCommandsStopped(String joint)throws Exception {
        var c=robot();String group="joint/"+joint;var setup=c.motionSetup();try(var demo=new RobotMotionDemo(setup,new DesktopAssetManager(true),group)) {
            assertEquals(2,demo.actions.size());for(int i=0;i<3000&&!demo.finished();i++){demo.tick();for(String name:setup.plan().driveNames)assertEquals(0,setup.hardware().get(simcore.SimDcMotorEx.class,name).getPower());}
            fixtures().complete(demo);assertEquals(List.of(group+"/first",group+"/second"),demo.observations.stream().map(r->r.get("id")).toList());for(var observation:demo.observations)assertEquals("movement observed",observation.get("outcome"),observation.toString());
        }
    }
    @Test void explicitReviewsAccumulateAndNewRetestRequiresAssessmentWithoutErasingOtherMovements()throws Exception {
        var c=robot();run(c,"");save(c,"drive/forward","Correct");save(c,"drive/backward","Reversed");var old=c.motionReview();
        assertEquals("Correct",row(old,"drive/forward").assessment());assertEquals("Reversed",row(old,"drive/backward").assessment());assertFalse(c.session.ready("robot"));assertFalse(Files.exists(c.session.modelPath("robot").getParent().resolve("validation.json")));
        run(c,"drive/forward");var fresh=c.motionReview();assertEquals("Not reviewed",row(fresh,"drive/forward").assessment());assertEquals("Reversed",row(fresh,"drive/backward").assessment());assertTrue(row(fresh,"drive/forward").reviewable());
        assertThrows(IllegalArgumentException.class,()->c.saveMotionReviews(old,Map.of("drive/forward","Correct"),Map.of()));save(c,"drive/forward","Correct");assertEquals("Correct",row(c.motionReview(),"drive/forward").assessment());
    }
    @Test void missingIncompleteAndNoMotionReportsCannotBecomeCorrectReviews()throws Exception {
        var c=robot();assertThrows(IllegalArgumentException.class,()->save(c,"drive/forward","Correct"));
        try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true))) {
            while(demo.observations.isEmpty())demo.tick();demo.stop();ProfileIO.save(c.motionReport(),demo.report());assertThrows(IllegalArgumentException.class,()->save(c,"drive/forward","Blocked"));
        }
        var setup=c.motionSetup();for(var motor:setup.hardware().getAll(simcore.SimDcMotorEx.class)){motor.setTargetPosition(0);motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);}
        try(var demo=new RobotMotionDemo(setup,new DesktopAssetManager(true),"drive/forward")){fixtures().complete(demo);ProfileIO.save(c.motionReport(),demo.report());assertEquals("no clear movement",demo.observations.get(0).get("outcome"));}
        assertThrows(IllegalArgumentException.class,()->save(c,"drive/forward","Correct"));save(c,"drive/forward","Blocked");assertEquals("Blocked",row(c.motionReview(),"drive/forward").assessment());
    }
    @Test void motionReviewsSaveAsAnotherImmutableRevisionAndSurvivePortableImportAndResume()throws Exception {
        var c=robot();ready(c);Path prior=c.session.modelPath("robot");String original=Files.readString(prior),proof=Files.readString(prior.getParent().resolve("validation.json"));String digest=c.motionSetup().profile().digest;
        run(c,"");save(c,"drive/forward","Correct");Path saved=c.session.modelPath("robot");assertNotEquals(prior,saved);assertTrue(c.session.ready("robot"));assertEquals(original,Files.readString(prior));assertEquals(proof,Files.readString(prior.getParent().resolve("validation.json")));assertEquals(digest,c.motionSetup().profile().digest);
        Path bundle=tmp.resolve("reviewed.zip");c.models.run("export",saved.toString(),bundle.toString());var restored=new GuidedSetupController(tmp.resolve("portable"),null);restored.session.start("robot",null);restored.load("robot","bundle",bundle,null);
        assertEquals("Correct",row(restored.motionReview(),"drive/forward").assessment());assertTrue(restored.session.ready("robot"));assertEquals(RobotMotionReview.entries(c.session.profile("robot")),RobotMotionReview.entries(restored.session.profile("robot")));
        var resumed=new GuidedSetupController(tmp.resolve("portable"),null);resumed.resume(restored.session.file);assertEquals("Correct",row(resumed.motionReview(),"drive/forward").assessment());
        Path beforeRetest=restored.session.modelPath("robot");String beforeText=Files.readString(beforeRetest);restored.prepareMotionRetest("drive/forward");assertNotEquals(beforeRetest,restored.session.modelPath("robot"));assertEquals(beforeText,Files.readString(beforeRetest));assertFalse(row(restored.motionReview(),"drive/forward").reviewable());assertEquals("Not reviewed",row(restored.motionReview(),"drive/forward").assessment());
    }
    @Test void changedCadRetainsHistoricalReviewsButRequiresFreshNativeMovementAndCollisionReview()throws Exception {
        var c=robot();run(c,"");save(c,"joint/slide_joint/first","Incorrect travel");ready(c);Path old=c.session.modelPath("robot");var review=RobotMotionReview.entries(c.session.profile("robot"));
        String xml=fixtures().fixture(true,true).replace(".3 0 .3",".32 0 .3");Path zip=tmp.resolve("changed-robot.zip");try(var out=new ZipOutputStream(Files.newOutputStream(zip))){out.putNextEntry(new ZipEntry("robot.urdf"));out.write(xml.getBytes(java.nio.charset.StandardCharsets.UTF_8));out.closeEntry();}
        c.load("robot","cad",zip,old);assertEquals(review,RobotMotionReview.entries(c.session.profile("robot")));assertFalse(c.session.ready("robot"));assertFalse(row(c.motionReview(),"joint/slide_joint/first").reviewable());assertThrows(IllegalArgumentException.class,()->save(c,"joint/slide_joint/first","Correct"));
        run(c,"joint/slide_joint");save(c,"joint/slide_joint/first","Correct");assertEquals("Correct",row(c.motionReview(),"joint/slide_joint/first").assessment());assertFalse(c.session.ready("robot"));
    }
    @Test void settingsAndHardwareChangesInvalidateOldAssessmentsAndCorrectionChoicesFindThePart()throws Exception {
        var c=robot();run(c,"");save(c,"drive/forward","Reversed");save(c,"joint/arm_joint/first","Incorrect travel");var snapshot=c.motionReview();
        assertEquals(List.of("wheel0","wheel1"),RobotMotionReviewDialog.parts(snapshot,row(snapshot,"drive/forward")));assertEquals(List.of("arm"),RobotMotionReviewDialog.parts(snapshot,row(snapshot,"joint/arm_joint/first")));
        assertEquals(RobotMotionReview.Correction.DRIVE,RobotMotionReview.recommended(row(snapshot,"drive/forward")));assertEquals(RobotMotionReview.Correction.JOINT,RobotMotionReview.recommended(row(snapshot,"joint/arm_joint/first")));assertTrue(RobotMotionReview.advice(row(snapshot,"joint/arm_joint/first"),RobotMotionReview.Correction.JOINT).contains("source length units"));
        var profile=c.session.profile("robot");FieldPackage.map(profile.get("parameters")).put("friction",.01);c.update("robot",profile);c.compile("robot");assertFalse(row(c.motionReview(),"drive/forward").reviewable());assertThrows(IllegalArgumentException.class,()->c.saveMotionReviews(snapshot,Map.of("drive/forward","Correct"),Map.of()));
        run(c,"drive/forward");save(c,"drive/forward","Correct");Path project=tmp.resolve("team");Files.createDirectories(project);
        String xml="<Robot><LynxUsbDevice name=\"hub\"><LynxModule name=\"module\">";int port=0;for(String name:List.of("leftDrive","rightDrive","armMotor","slideMotor"))xml+="<Motor name=\""+name+"\" port=\""+(port++)+"\"/>";xml+="<Servo name=\"gateServo\" port=\"0\"/></LynxModule></LynxUsbDevice></Robot>";
        Files.writeString(project.resolve("robot.xml"),xml);ProfileIO.save(project.resolve("motors.json"),Map.of("name","test","motors",Map.of()));ProfileIO.save(project.resolve("sim.config"),Map.of("robotConfig","robot.xml","presetMotors","motors.json"));c.session.start("robot",project);assertFalse(row(c.motionReview(),"drive/forward").reviewable());
        assertTrue(c.motionSetup().hardwareLabel().contains("generic fallback motor specs for leftDrive, rightDrive, armMotor, slideMotor"));
        run(c,"drive/forward");save(c,"drive/forward","Correct");Files.writeString(project.resolve("motors.json"),Files.readString(project.resolve("motors.json"))+"\n");assertFalse(row(c.motionReview(),"drive/forward").reviewable());
    }
}
