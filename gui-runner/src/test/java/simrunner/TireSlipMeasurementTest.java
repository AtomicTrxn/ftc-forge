package simrunner;

import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;
import java.nio.file.*;
import java.util.*;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.system.NativeLibraryLoader;
import physics.TireFriction;
import physics.calibration.TireSlipCalibrator.*;
import simcore.MiniJson;

class TireSlipMeasurementTest {
    @TempDir Path tmp;
    @BeforeAll static void nativePhysics(){NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    static final TireFriction.Spec TRUE=new TireFriction.Spec(.9,.6,1,100,.15);
    static List<TireSlipMeasurement.Row> data(){var rows=new ArrayList<TireSlipMeasurement.Row>();for(String set:List.of("fit","validate"))for(double load:set.equals("fit")?new double[]{2,5,20}:new double[]{3,8,15})for(double v:set.equals("fit")?new double[]{.003,.012,.035,.075,.12,.22,.45,.9}:new double[]{.005,.02,.055,.1,.17,.32,.7})for(int sign:new int[]{-1,1}){double slip=sign*v;rows.add(new TireSlipMeasurement.Row("wheel_joint0",new Reading(set+rows.size(),set,.3+slip,.3,load,TireFriction.steadyLongitudinal(TRUE,slip,load))));}return rows;}
    Map<String,Object> evidence()throws Exception{return TireSlipMeasurement.evidence(new TireSlipMeasurement.Context("traction","Synthetic physical-method fixture","Independent optical fixture","Independent force fixture"),data(),1,Bounds.defaults(),Criteria.defaults());}
    GuidedSetupController model()throws Exception {var f=new RobotMotionDemoTest();f.tmp=tmp;var c=f.model(true,false);var p=c.session.profile("robot");var runtime=FieldPackage.map(p.get("runtime"));runtime.put("tires",Map.of("traction",TireSlipMeasurement.spec(TRUE),"omni",TireSlipMeasurement.spec(new TireFriction.Spec(.8,.5,.05,80,.2)),"omni_joints",List.of("wheel_joint1"),"reflected_motor_inertia_kg_m2",.0015,"contact_tolerance_m",.004));c.update("robot",p);c.compile("robot");return c;}
    @Test void csvRoundTripsQuotesAndRejectsMalformedOrWrongClassReadings()throws Exception {
        var rows=data();Path file=tmp.resolve("recording.csv");Files.writeString(file,TireSlipMeasurement.csv(rows));assertEquals(rows,TireSlipMeasurement.readCsv(file,Set.of("wheel_joint0")));
        var quoted=List.of(new TireSlipMeasurement.Row("wheel,\"left\"",rows.get(0).reading()));Files.writeString(file,TireSlipMeasurement.csv(quoted));assertEquals(quoted,TireSlipMeasurement.readCsv(file,Set.of("wheel,\"left\"")));
        TireSlipMeasurement.template(file,"wheel_joint0");assertTrue(Files.readString(file).contains("fit-12,fit,,,,"));assertThrows(IllegalArgumentException.class,()->TireSlipMeasurement.readCsv(file,Set.of("wheel_joint0")));
        String good=TireSlipMeasurement.csv(rows);for(String bad:List.of(good.replace("hub_speed_mps","encoder_hub_speed"),good.replace("2.0,", "NaN,"),good+"wheel_joint0,fit0,validate,1,0,2,1\n",good+"wheel_joint1,fit0,fit,1,0,2,1\n")){Files.writeString(file,bad);assertThrows(IllegalArgumentException.class,()->TireSlipMeasurement.readCsv(file,Set.of("wheel_joint0")));}
        Files.writeString(file,good);assertThrows(IllegalArgumentException.class,()->TireSlipMeasurement.readCsv(file,Set.of("other")));
    }
    @Test void rechecksCanonicalEvidenceAndRejectsStaleContextResultsAndParameters()throws Exception {
        var e=evidence();var spec=TireSlipMeasurement.fit(data(),1,Bounds.defaults(),Criteria.defaults()).spec();assertEquals(data(),TireSlipMeasurement.check(MiniJson.parseObject(ProfileIO.json(e)),"traction",spec).rows());
        assertThrows(IllegalArgumentException.class,()->TireSlipMeasurement.check(e,"omni",spec));assertThrows(IllegalArgumentException.class,()->TireSlipMeasurement.check(e,"traction",new TireFriction.Spec(.9,.6,1,120,.15)));
        for(String change:List.of("readings","surface","criteria","results")){var copy=MiniJson.parseObject(ProfileIO.json(e));switch(change){case "readings"->FieldPackage.maps(copy.get("readings")).get(0).put("normal_load_n",3.);case "surface"->copy.put("reference_surface","changed");case "criteria"->FieldPackage.map(copy.get("criteria")).put("max_relative_rmse",.2);default->FieldPackage.map(copy.get("result")).put("validation_rmse_n",3.);}assertThrows(IllegalArgumentException.class,()->TireSlipMeasurement.check(copy,"traction",spec));}
    }
    @Test void explicitApplicationRunsNativeTiresAndPreservesBundlesAndOriginals()throws Exception {
        var c=model();var validator=new GuidedSetupTest();validator.tmp=tmp;validator.ready(c,"robot");Path original=c.session.modelPath("robot");String before=Files.readString(original),context=c.motionSetup().context();var e=evidence();
        assertThrows(IllegalArgumentException.class,()->c.measuredTireSlip("traction",e,false));assertThrows(IllegalArgumentException.class,()->c.measuredTireSlip("omni",e,true));
        var wrongWheels=TireSlipMeasurement.evidence(new TireSlipMeasurement.Context("omni","Reference","Independent optics","Direct force sensor"),data(),.05,Bounds.defaults(),Criteria.defaults());assertThrows(IllegalArgumentException.class,()->c.measuredTireSlip("omni",wrongWheels,true));
        c.measuredTireSlip("traction",e,true);assertFalse(c.session.ready("robot"));assertNotEquals(context,c.motionSetup().context());assertEquals(before,Files.readString(original));
        assertEquals(.05,c.motionSetup().config().tires.omni().lateralScale());assertEquals(.0015,c.motionSetup().config().tires.reflectedMotorInertiaKgM2());
        var f=new RobotMotionDemoTest();f.tmp=tmp;try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")){f.complete(demo);assertTrue(FieldPackage.num(demo.observations.get(0),"forward_m")>.005);assertNotNull(demo.world.tireDrive());}
        validator.ready(c,"robot");Path bundle=tmp.resolve("tire.zip");c.models.run("export",c.session.modelPath("robot").toString(),bundle.toString());var restored=new GuidedSetupController(tmp.resolve("portable"),null);restored.session.start("robot",null);restored.load("robot","bundle",bundle,null);assertTrue(restored.session.ready("robot"));var snapshot=TireSlipCalibrationGuide.read(restored,"traction");assertEquals(data(),TireSlipMeasurement.check(snapshot.evidence(),"traction",snapshot.spec()).rows());assertEquals(before,Files.readString(original));
    }
    @Test void cadRenamePreservesHistoricalReadingsAndRequiresReview()throws Exception {
        var c=model();c.measuredTireSlip("traction",evidence(),true);var validator=new GuidedSetupTest();validator.tmp=tmp;validator.ready(c,"robot");Path saved=c.session.modelPath("robot");String original=Files.readString(saved);var fixture=new RobotMotionDemoTest();fixture.tmp=tmp;
        Path zip=tmp.resolve("new.zip");try(var out=new java.util.zip.ZipOutputStream(Files.newOutputStream(zip))){out.putNextEntry(new java.util.zip.ZipEntry("robot.urdf"));out.write(fixture.fixture(true,false).replace("wheel_joint0","renamed_wheel").getBytes(java.nio.charset.StandardCharsets.UTF_8));out.closeEntry();}
        c.load("robot","cad",zip,saved);var pending=FieldPackage.maps(FieldPackage.map(c.session.profile("robot").get("migration")).get("pending"));while(!pending.isEmpty()){assertTrue(((List<?>)pending.get(0).get("options")).contains("Keep saved bindings"));c.resolve("robot",0,"Keep saved bindings");pending=FieldPackage.maps(FieldPackage.map(c.session.profile("robot").get("migration")).get("pending"));}
        assertFalse(c.session.ready("robot"));assertEquals(List.of("renamed_wheel"),c.tireClassChoices().get("traction"));var snapshot=TireSlipCalibrationGuide.read(c,"traction");assertEquals(data(),TireSlipMeasurement.check(snapshot.evidence(),"traction",snapshot.spec()).rows());assertEquals(original,Files.readString(saved));
    }
    @Test void manualEditsCannotUseStaleEvidenceAndDiscardKeepsOtherClass()throws Exception {
        var c=model();c.measuredTireSlip("traction",evidence(),true);var p=c.session.profile("robot");var tires=FieldPackage.map(FieldPackage.map(p.get("runtime")).get("tires"));FieldPackage.map(tires.get("traction")).put("stiffness_n_per_mps",120.);c.update("robot",p);assertThrows(IllegalArgumentException.class,()->c.compile("robot"));var other=ProfileIO.json(tires.get("omni"));c.discardTireMeasurements("traction");assertEquals(120.,c.motionSetup().config().tires.traction().stiffnessNPerMps());assertTrue(TireSlipCalibrationGuide.read(c,"traction").evidence().isEmpty());assertEquals(other,ProfileIO.json(FieldPackage.map(FieldPackage.map(c.session.profile("robot").get("runtime")).get("tires")).get("omni")));assertFalse(c.session.ready("robot"));
    }
    @Test void unconfiguredModelsCannotBeSilentlyEnabled()throws Exception {var f=new RobotMotionDemoTest();f.tmp=tmp;var c=f.model(true,false);assertThrows(IllegalArgumentException.class,c::tireClassChoices);assertThrows(IllegalArgumentException.class,()->c.measuredTireSlip("traction",evidence(),true));assertFalse(FieldPackage.map(c.session.profile("robot").get("runtime")).containsKey("tires"));}
    @Test void multipleStaleClassesCanBeCorrectedIndividuallyWhilePhysicsStaysBlocked()throws Exception {
        var c=model();c.measuredTireSlip("traction",evidence(),true);
        var otherRows=data().stream().map(r->new TireSlipMeasurement.Row("wheel_joint1",r.reading())).toList();
        var other=TireSlipMeasurement.evidence(new TireSlipMeasurement.Context("omni","Reference","Independent optics","Direct force sensor"),otherRows,.05,Bounds.defaults(),Criteria.defaults());c.measuredTireSlip("omni",other,true);
        var p=c.session.profile("robot");var tires=FieldPackage.map(FieldPackage.map(p.get("runtime")).get("tires"));for(String group:List.of("traction","omni"))FieldPackage.map(FieldPackage.map(FieldPackage.map(tires.get(group)).get("measurement")).get("result")).put("validation_rmse_n",3.);c.update("robot",p);
        assertTrue(assertThrows(IllegalArgumentException.class,()->c.discardTireMeasurements("traction")).getMessage().contains("omni"));
        assertTrue(TireSlipCalibrationGuide.read(c,"traction").evidence().isEmpty());assertFalse(TireSlipCalibrationGuide.read(c,"omni").evidence().isEmpty());assertThrows(IllegalArgumentException.class,c::motionSetup);
        c.discardTireMeasurements("omni");assertNotNull(c.motionSetup().config().tires);assertFalse(c.session.ready("robot"));
    }
}
