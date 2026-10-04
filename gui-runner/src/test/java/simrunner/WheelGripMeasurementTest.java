package simrunner;

import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;
import java.nio.file.*;
import java.util.*;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.system.NativeLibraryLoader;
import physics.calibration.GripCalibrator.*;
import simcore.MiniJson;

class WheelGripMeasurementTest {
    @TempDir Path tmp;
    @BeforeAll static void nativePhysics(){NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    List<Reading> data(){return List.of(new Reading("fit1","fit",10,4.5),new Reading("fit2","fit",20,9),new Reading("fit3","fit",30,13.5),new Reading("v1","validate",15,6.75),new Reading("v2","validate",25,11.25));}
    Map<String,Object> evidence()throws Exception{return WheelGripMeasurement.evidence("wheel_joint0","Physical test surface",.6,Criteria.defaults(),data());}
    @Test void csvUnitsAndQuotedNamesConvertToTheSamePhysicalCoefficient()throws Exception {
        String joint="wheel,\"left\"";for(var load:WheelGripMeasurement.LoadUnit.values())for(var force:WheelGripMeasurement.ForceUnit.values()) {
            var csv=new StringBuilder("joint,trial,set,"+load.column+","+force.column+"\n");for(var r:data())csv.append("\"wheel,\"\"left\"\"\",").append(r.trial()).append(',').append(r.set()).append(',').append(r.normalN()/load.newtons).append(',').append(r.forceN()/force.newtons).append('\n');
            Path file=tmp.resolve(load.name()+force.name()+".csv");Files.writeString(file,csv);var rows=WheelGripMeasurement.readCsv(file,joint);assertEquals(0,rows.otherWheelRows());var e=WheelGripMeasurement.evidence(joint,"Reference",.6,Criteria.defaults(),rows.readings());assertEquals(.75,FieldPackage.num(FieldPackage.map(e.get("result")),"wheel_coefficient"),1e-12);
            Files.writeString(file,WheelGripMeasurement.normalizedCsv(joint,rows.readings()));assertEquals(rows.readings(),WheelGripMeasurement.readCsv(file,joint).readings());
        }
    }
    @Test void csvRejectsWrongUnitsTrialLeakageMalformedNumbersAndWrongTarget()throws Exception {
        Path file=tmp.resolve("bad.csv");String good=WheelGripMeasurement.normalizedCsv("wheel_joint0",data());
        for(String csv:List.of(good.replace("normal_load_n","load"),good.replace("normal_load_n,sliding_force_n","normal_load_n,normal_mass_kg"),good.replace("10.0,4.5","NaN,4.5"),good+"wheel_joint0,fit1,validate,10,4.5\n",good+"\"broken,trial,fit,10,4.5\n")){Files.writeString(file,csv);assertThrows(IllegalArgumentException.class,()->WheelGripMeasurement.readCsv(file,"wheel_joint0"));}
        Files.writeString(file,good);assertThrows(IllegalArgumentException.class,()->WheelGripMeasurement.readCsv(file,"different"));Files.writeString(file,good+"other,extra,fit,20,9\n");assertEquals(1,WheelGripMeasurement.readCsv(file,"wheel_joint0").otherWheelRows());
        WheelGripMeasurement.template(file,"wheel_joint0",WheelGripMeasurement.LoadUnit.KG,WheelGripMeasurement.ForceUnit.LBF);assertTrue(Files.readString(file).contains("normal_mass_kg,sliding_force_lbf"));assertThrows(IllegalArgumentException.class,()->WheelGripMeasurement.readCsv(file,"wheel_joint0"),"Blank templates cannot be mistaken for measured data");
    }
    @Test void evidenceRecomputesResultsAndRejectsChangedReadingsCriteriaOrCoefficient()throws Exception {
        var e=evidence();assertEquals(.75,WheelGripMeasurement.check(e,.75).result().wheelCoefficient());
        var parsed=MiniJson.parseObject(ProfileIO.json(e));assertEquals(data(),WheelGripMeasurement.check(parsed,.75).readings());assertThrows(IllegalArgumentException.class,()->WheelGripMeasurement.check(e,.8));
        var rows=FieldPackage.maps(parsed.get("readings"));rows.get(0).put("sliding_force_n",4.6);var readingsChanged=parsed;assertTrue(assertThrows(IllegalArgumentException.class,()->WheelGripMeasurement.check(readingsChanged,.75)).getMessage().contains("changed"));
        parsed=MiniJson.parseObject(ProfileIO.json(e));FieldPackage.map(parsed.get("result")).put("validation_rmse_n",1.);var resultChanged=parsed;assertThrows(IllegalArgumentException.class,()->WheelGripMeasurement.check(resultChanged,.75));
        parsed=MiniJson.parseObject(ProfileIO.json(e));FieldPackage.map(parsed.get("criteria")).put("min_validation_trials",3);var criteriaChanged=parsed;assertThrows(IllegalArgumentException.class,()->WheelGripMeasurement.check(criteriaChanged,.75));
        parsed=MiniJson.parseObject(ProfileIO.json(e));FieldPackage.map(parsed.get("criteria")).put("max_relative_rmse",.2);var relaxedCriteria=parsed;assertTrue(assertThrows(IllegalArgumentException.class,()->WheelGripMeasurement.check(relaxedCriteria,.75)).getMessage().contains("quality criteria changed"));
    }
    @Test void measuredApplicationIsExplicitPortableAndDrivesNativePhysics()throws Exception {
        var f=new RobotMotionDemoTest();f.tmp=tmp;var c=f.model(true,false);var e=evidence();var ready=new GuidedSetupTest();ready.tmp=tmp;ready.ready(c,"robot");Path saved=c.session.modelPath("robot");String original=Files.readString(saved),context=c.motionSetup().context();
        assertThrows(IllegalArgumentException.class,()->c.measuredWheelGrip("wheel_joint0",e,false));assertThrows(IllegalArgumentException.class,()->c.measuredWheelGrip("wheel_joint1",e,true));assertEquals(saved,c.session.modelPath("robot"));
        c.measuredWheelGrip("wheel_joint0",e,true);assertFalse(c.session.ready("robot"));assertNotEquals(context,c.motionSetup().context());assertEquals(original,Files.readString(saved));assertEquals(MiniJson.parseObject(ProfileIO.json(e)),WheelGripCalibrationGuide.read(c,"wheel_joint0").evidence());
        try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")){f.complete(demo);assertTrue(FieldPackage.num(demo.observations.get(0),"forward_m")>.03);}
        ready.ready(c,"robot");Path bundle=tmp.resolve("measured.zip");c.models.run("export",c.session.modelPath("robot").toString(),bundle.toString());var restored=new GuidedSetupController(tmp.resolve("portable"),null);restored.session.start("robot",null);restored.load("robot","bundle",bundle,null);assertTrue(restored.session.ready("robot"));
        var snapshot=WheelGripCalibrationGuide.read(restored,"wheel_joint0");assertEquals(data(),WheelGripMeasurement.check(snapshot.evidence(),snapshot.currentCoefficient()).readings());assertTrue(FieldPackage.map(restored.session.profile("robot").get("provenance")).get("runtime/drive_contacts/wheel_friction/0/friction").toString().contains("validation passed"));
        restored.wheelGrip("wheel_joint0",.75);assertTrue(restored.session.ready("robot"));assertFalse(WheelGripCalibrationGuide.read(restored,"wheel_joint0").evidence().isEmpty());
        restored.wheelGrip("wheel_joint0",.8);assertFalse(restored.session.ready("robot"));assertTrue(WheelGripCalibrationGuide.read(restored,"wheel_joint0").evidence().isEmpty());assertEquals(original,Files.readString(saved));
    }
    @Test void staleEvidenceCanBeDiscardedWithoutChangingTheCoefficientOrAnotherWheel()throws Exception {
        var f=new RobotMotionDemoTest();f.tmp=tmp;var c=f.model(true,false);c.measuredWheelGrip("wheel_joint0",evidence(),true);c.wheelGrip("wheel_joint1",.4);
        var p=c.session.profile("robot");var entries=FieldPackage.maps(FieldPackage.map(FieldPackage.map(p.get("runtime")).get("drive_contacts")).get("wheel_friction"));FieldPackage.map(FieldPackage.map(entries.get(0).get("measurement")).get("result")).put("validation_rmse_n",3.);c.update("robot",p);assertThrows(IllegalArgumentException.class,()->c.compile("robot"));
        c.wheelGrip("wheel_joint0",.75,true);c.compile("robot");assertTrue(WheelGripCalibrationGuide.read(c,"wheel_joint0").evidence().isEmpty());assertEquals(.4,c.motionSetup().config().driveContacts.wheelFriction().get("wheel_joint1"));
    }
    @Test void renamedCadJointKeepsMeasurementHistoryAndRequiresFreshReview()throws Exception {
        var f=new RobotMotionDemoTest();f.tmp=tmp;var c=f.model(true,false);c.measuredWheelGrip("wheel_joint0",evidence(),true);var validator=new GuidedSetupTest();validator.tmp=tmp;validator.ready(c,"robot");Path saved=c.session.modelPath("robot");String before=Files.readString(saved);
        Path zip=tmp.resolve("renamed.zip");try(var out=new java.util.zip.ZipOutputStream(Files.newOutputStream(zip))){out.putNextEntry(new java.util.zip.ZipEntry("model.urdf"));out.write(f.fixture(true,false).replace("wheel_joint0","renamed_wheel_joint").getBytes(java.nio.charset.StandardCharsets.UTF_8));out.closeEntry();}
        c.load("robot","cad",zip,saved);assertFalse(c.session.ready("robot"));var pending=FieldPackage.maps(FieldPackage.map(c.session.profile("robot").get("migration")).get("pending"));
        while(!pending.isEmpty()){assertTrue(((List<?>)pending.get(0).get("options")).contains("Keep saved bindings"),pending.toString());c.resolve("robot",0,"Keep saved bindings");pending=FieldPackage.maps(FieldPackage.map(c.session.profile("robot").get("migration")).get("pending"));}
        var snapshot=WheelGripCalibrationGuide.read(c,"renamed_wheel_joint");assertEquals(.75,snapshot.currentCoefficient(),1e-12);assertEquals("wheel_joint0",snapshot.evidence().get("recorded_joint"));assertEquals(data(),WheelGripMeasurement.check(snapshot.evidence(),.75).readings());assertFalse(c.session.ready("robot"));assertEquals(before,Files.readString(saved));
        assertEquals(Map.of("renamed_wheel_joint",.75),c.motionSetup().config().driveContacts.wheelFriction());
    }
    @Test void multipleStaleWheelsCanBeCorrectedOneAtATimeWithoutEnablingPhysics()throws Exception {
        var f=new RobotMotionDemoTest();f.tmp=tmp;var c=f.model(true,false);
        c.measuredWheelGrip("wheel_joint0",evidence(),true);
        c.measuredWheelGrip("wheel_joint1",WheelGripMeasurement.evidence("wheel_joint1","Reference",.6,Criteria.defaults(),data()),true);
        var p=c.session.profile("robot");var entries=FieldPackage.maps(FieldPackage.map(FieldPackage.map(p.get("runtime")).get("drive_contacts")).get("wheel_friction"));
        for(var entry:entries)FieldPackage.map(FieldPackage.map(entry.get("measurement")).get("result")).put("validation_rmse_n",3.);
        c.update("robot",p);
        assertTrue(assertThrows(IllegalArgumentException.class,()->c.wheelGrip("wheel_joint0",.75,true)).getMessage().contains("wheel_joint1"));
        assertTrue(WheelGripCalibrationGuide.read(c,"wheel_joint0").evidence().isEmpty());
        assertFalse(WheelGripCalibrationGuide.read(c,"wheel_joint1").evidence().isEmpty());
        assertFalse(c.session.ready("robot"));assertThrows(IllegalArgumentException.class,()->c.motionSetup());
        c.wheelGrip("wheel_joint1",.75,true);c.compile("robot");
        assertEquals(Map.of("wheel_joint0",.75,"wheel_joint1",.75),c.motionSetup().config().driveContacts.wheelFriction());
    }
}
