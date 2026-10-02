package simrunner;

import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import static org.junit.jupiter.api.Assertions.*;
import java.nio.file.*;
import java.util.*;
import java.util.zip.*;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.system.NativeLibraryLoader;
import simcore.*;

class RobotMeasurementsTest {
    @TempDir Path tmp;
    @BeforeAll static void nativeLibrary(){NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    GuidedSetupController controller()throws Exception{
        var c=new GuidedSetupController(tmp.resolve("library"),null);c.session.start("simulation",null);
        Path zip=tmp.resolve("robot.zip");try(var z=new ZipOutputStream(Files.newOutputStream(zip))){z.putNextEntry(new ZipEntry("model.urdf"));z.write(GuidedSetupTest.ROBOT.getBytes(java.nio.charset.StandardCharsets.UTF_8));z.closeEntry();}
        c.load("robot","cad",zip,null);return c;
    }
    void ready(GuidedSetupController c)throws Exception{
        try(var built=new ModelValidation.Prepared(c.session.modelPath("robot"),new DesktopAssetManager(true))){built.writeProof();}
        c.session.select("robot",Path.of(c.models.run("save",c.session.modelPath("robot").toString(),c.models.library.toString(),"--reviewed").toString()));
    }
    Path source(String xml)throws Exception{
        Path zip=tmp.resolve(UUID.randomUUID()+".zip");
        try(var z=new ZipOutputStream(Files.newOutputStream(zip))){z.putNextEntry(new ZipEntry("model.urdf"));z.write(xml.getBytes(java.nio.charset.StandardCharsets.UTF_8));z.closeEntry();}
        return zip;
    }
    String boundRobot(){
        return "<robot name=\"Measured robot\"><link name=\"base\"><visual><geometry><box size=\".3 .2 .1\"/></geometry></visual></link>"
            +"<link name=\"wheel_left\"><visual><geometry><sphere radius=\".04\"/></geometry></visual></link>"
            +"<joint name=\"left_joint\" type=\"continuous\"><parent link=\"base\"/><child link=\"wheel_left\"/><origin xyz=\"0 .2 0\"/><axis xyz=\"0 1 0\"/></joint>"
            +"<transmission name=\"left_tx\"><joint name=\"left_joint\"/><actuator name=\"leftDrive\"><mechanicalReduction>1</mechanicalReduction></actuator></transmission></robot>";
    }
    void assertPortableMeasurements(GuidedSetupController c,boolean differential,Map<String,Object> expectedRuntime)throws Exception{
        var p=c.session.profile("robot");assertEquals(expectedRuntime,p.get("runtime"));
        var selection=Map.<String,Object>of("robot_model_profile",c.session.modelPath("robot").toString(),"model_settings","profile");
        SimConfig config;
        if(c.session.ready("robot"))config=SimConfig.parse(tmp,selection);
        else {
            assertThrows(java.io.IOException.class,()->SimConfig.parse(tmp,selection),"Migrated drafts must not become runnable without review.");
            var model=c.session.checked("robot");var prepared=new LinkedHashMap<>(model.runtime);prepared.put("urdf",model.artifact("robot").toString());config=SimConfig.parse(model.directory,prepared);
        }
        assertEquals(9.0718474,config.totalMassKg,1e-9);
        if(differential){assertEquals(.0508,config.drive.wheelRadiusM(),1e-9);assertEquals(.4064,config.drive.trackWidthM(),1e-9);assertEquals("leftDrive",config.drive.leftMotor());assertEquals("rightDrive",config.drive.rightMotor());assertEquals(-1,config.drive.leftShaftSign());assertEquals(1,config.drive.rightShaftSign());}
        else {assertEquals(.0508,config.driveGeometry.wheelRadiusM(),1e-9);assertEquals(.4064,config.driveGeometry.trackWidthM(),1e-9);assertEquals(.4572,config.driveGeometry.wheelbaseM(),1e-9);}
        var calibration=CalibrationProfile.load(c.session.modelPath("robot").getParent().resolve(config.calibration));assertEquals(12.7,calibration.vInternal);assertEquals(.15,calibration.rBattery);assertEquals(.03,calibration.motors.get("leftDrive").staticTorqueNm());assertEquals(.004,calibration.motors.get("leftDrive").viscousBNmS());assertEquals(.2,calibration.drive.responseTimeS());
        var urdf=RobotUrdf.parse(c.session.checked("robot").artifact("robot"));var binding=urdf.transmissions.values().stream().flatMap(t->t.actuators().stream()).filter(a->a.name().equals("leftDrive")).findFirst().orElseThrow();assertEquals(-2.,binding.mechanicalReduction());
        String part=FieldPackage.map(p.get("entities")).containsKey("wheel_front")?"wheel_front":"wheel_left";
        var provenance=FieldPackage.map(p.get("provenance"));assertEquals("user supplied",provenance.get("entities/"+part+"/settings/actuators/0/name"));assertEquals("measured manually: sprocket ratio",provenance.get("entities/"+part+"/settings/actuators/0/mechanicalReduction"));
        for(String key:differential?List.of("total_mass_kg","drive/wheel_radius_m","drive/track_width_m"):List.of("total_mass_kg","drive_geometry/wheel_radius_m","drive_geometry/track_width_m","drive_geometry/wheelbase_m"))assertTrue(provenance.get("runtime/"+key).toString().startsWith("measured manually"));
        var guide=RobotMeasurementGuide.read(c);assertEquals(differential,guide.differential());assertEquals(.1016,guide.diameterM(),1e-9);assertEquals(.4064,guide.trackM(),1e-9);assertTrue(guide.bindings().stream().anyMatch(row->row.get(2).equals("leftDrive")&&row.get(3).equals("-2.0")));
    }
    @ParameterizedTest(name="portable measurements and CAD migration: differential={0}")
    @ValueSource(booleans={false,true})
    void guideMeasurementsAndBindingsSurvivePortableImportAndCadMigration(boolean differential)throws Exception{
            String variant=differential?"differential":"mecanum";var c=new GuidedSetupController(tmp.resolve(variant+"-library"),null);c.session.start("robot",null);String xml=boundRobot();c.load("robot","cad",source(xml),null);
            var p=c.session.profile("robot");var runtime=FieldPackage.map(p.get("runtime"));
            if(differential)runtime.put("drive",Map.of("type","differential","left_motor","leftDrive","right_motor","rightDrive","track_width_m",.38,"wheel_radius_m",.045,"left_shaft_sign",-1,"right_shaft_sign",1));
            runtime.put("calibration_data",Map.of("battery",Map.of("v_internal",12.7,"r_battery",.15),"motors",Map.of("leftDrive",Map.of("tau_static_nm",.03,"viscous_b_nm_s_per_rad",.004)),"drive",Map.of("response_time_s",.2,"max_accel_mps2",2.,"yaw_response_time_s",.15,"max_yaw_accel_radps2",4.)));
            FieldPackage.map(FieldPackage.map(FieldPackage.map(p.get("entities")).get("wheel_left")).get("settings")).put("actuators",List.of(Map.of("name","leftDrive","mechanicalReduction",-2.)));
            var provenance=FieldPackage.map(p.get("provenance"));provenance.put("entities/wheel_left/settings/actuators/0/name","user supplied");provenance.put("entities/wheel_left/settings/actuators/0/mechanicalReduction","measured manually: sprocket ratio");c.update("robot",p);
            c.measurements(new RobotMeasurements.Entry(20*RobotMeasurements.MassUnit.LB.kg,4*RobotMeasurements.LengthUnit.IN.meters,16*RobotMeasurements.LengthUnit.IN.meters,differential?null:18*RobotMeasurements.LengthUnit.IN.meters));ready(c);
            Path saved=c.session.modelPath("robot");String original=Files.readString(saved);var expectedRuntime=FieldPackage.map(c.session.profile("robot").get("runtime"));
            Path bundle=tmp.resolve(variant+".zip");c.models.run("export",saved.toString(),bundle.toString());
            var imported=new GuidedSetupController(tmp.resolve(variant+"-portable-library"),null);imported.session.start("robot",null);imported.load("robot","bundle",bundle,null);assertTrue(imported.session.ready("robot"));assertPortableMeasurements(imported,differential,expectedRuntime);assertEquals(c.session.profile("robot").get("provenance"),imported.session.profile("robot").get("provenance"));
            Path portable=imported.session.modelPath("robot");String portableOriginal=Files.readString(portable);
            String renamed=xml.replace("wheel_left","wheel_front").replace("0 .2 0","0 .22 0");imported.load("robot","cad",source(renamed),portable);assertFalse(imported.session.ready("robot"));assertTrue(FieldPackage.maps(FieldPackage.map(imported.session.profile("robot").get("migration")).get("pending")).isEmpty());assertPortableMeasurements(imported,differential,expectedRuntime);
            assertFalse(FieldPackage.map(imported.session.profile("robot").get("provenance")).keySet().stream().anyMatch(key->key.startsWith("entities/wheel_left/")));assertThrows(Exception.class,()->imported.models.run("save",imported.session.modelPath("robot").toString(),imported.models.library.toString(),"--reviewed"));ready(imported);
            Path renamedSaved=imported.session.modelPath("robot");String renamedOriginal=Files.readString(renamedSaved);imported.load("robot","cad",source(renamed.replace("radius=\".04\"","radius=\".06\"")),renamedSaved);
            var pending=FieldPackage.maps(FieldPackage.map(imported.session.profile("robot").get("migration")).get("pending"));int index=java.util.stream.IntStream.range(0,pending.size()).filter(i->"wheel_front".equals(pending.get(i).get("new"))).findFirst().orElseThrow();assertTrue(((List<?>)pending.get(index).get("options")).contains("Use settings: wheel_front"));assertThrows(Exception.class,()->imported.models.run("save",imported.session.modelPath("robot").toString(),imported.models.library.toString(),"--reviewed"));imported.resolve("robot",index,"Use settings: wheel_front");assertFalse(imported.session.ready("robot"));assertPortableMeasurements(imported,differential,expectedRuntime);
            try(var built=new ModelValidation.Prepared(imported.session.modelPath("robot"),new DesktopAssetManager(true))){assertEquals(9.0718474,built.space.getRigidBodyList().stream().mapToDouble(b->b.getMass()).sum(),.0001);}ready(imported);
            Path secondBundle=tmp.resolve(variant+"-migrated.zip");imported.models.run("export",imported.session.modelPath("robot").toString(),secondBundle.toString());var restored=new GuidedSetupController(tmp.resolve(variant+"-final-library"),null);restored.session.start("robot",null);restored.load("robot","bundle",secondBundle,null);assertTrue(restored.session.ready("robot"));assertPortableMeasurements(restored,differential,expectedRuntime);assertEquals(imported.session.profile("robot").get("provenance"),restored.session.profile("robot").get("provenance"));
            assertEquals(original,Files.readString(saved));assertEquals(portableOriginal,Files.readString(portable));assertEquals(renamedOriginal,Files.readString(renamedSaved));
    }
    @Test void selectiveDifferentialMeasurementsKeepBindingsCalibrationAndUnselectedValues()throws Exception{
        var c=controller();var p=c.session.profile("robot");var runtime=FieldPackage.map(p.get("runtime"));
        runtime.put("drive",Map.of("type","differential","left_motor","leftDrive","right_motor","rightDrive","track_width_m",.38,"wheel_radius_m",.045,"left_shaft_sign",-1,"right_shaft_sign",1));
        runtime.put("calibration_data",Map.of("battery",Map.of("v_internal",12.7,"r_battery",.15),"motors",Map.of()));
        String original=ProfileIO.json(p);
        var changed=RobotMeasurements.apply(p,new RobotMeasurements.Entry(20*RobotMeasurements.MassUnit.LB.kg,4*RobotMeasurements.LengthUnit.IN.meters,.4,null));
        var result=FieldPackage.map(changed.get("runtime"));var drive=DifferentialDriveConfig.parse(FieldPackage.map(result.get("drive")));
        assertEquals(9.0718474,FieldPackage.num(result,"total_mass_kg"),1e-9);assertEquals(.0508,drive.wheelRadiusM(),1e-9);assertEquals(.4,drive.trackWidthM());
        assertEquals("leftDrive",drive.leftMotor());assertEquals(-1,drive.leftShaftSign());assertEquals(runtime.get("calibration_data"),result.get("calibration_data"));assertEquals(original,ProfileIO.json(p));
        var massOnly=RobotMeasurements.apply(changed,new RobotMeasurements.Entry(10.,null,null,null));
        assertEquals(result.get("drive"),FieldPackage.map(massOnly.get("runtime")).get("drive"));
        assertTrue(FieldPackage.map(massOnly.get("provenance")).get("runtime/drive/wheel_radius_m").toString().startsWith("measured manually"));
    }
    @Test void invalidMeasurementsAndAmbiguousDriveGeometryAreRejectedBeforeSaving()throws Exception{
        var c=controller();Path file=c.session.modelPath("robot");String original=Files.readString(file);
        for(double invalid:new double[]{0,-1,Double.NaN,Double.POSITIVE_INFINITY})assertThrows(IllegalArgumentException.class,()->new RobotMeasurements.Entry(invalid,null,null,null));
        assertThrows(IllegalArgumentException.class,()->c.measurements(new RobotMeasurements.Entry(null,.09,null,null)));
        assertEquals(original,Files.readString(file));
        for(double invalid:new double[]{0,-1,Double.NaN,Double.POSITIVE_INFINITY})assertThrows(IllegalArgumentException.class,()->SimConfig.parse(tmp,Map.of("total_mass_kg",invalid)));
        assertThrows(IllegalArgumentException.class,()->SimConfig.parse(tmp,Map.of("drive_geometry",Map.of("wheel_radius_m",.05,"track_width_m",.4))));
        var drive=Map.of("type","differential","left_motor","left","right_motor","right","track_width_m",.4,"wheel_radius_m",.05,"left_shaft_sign",1,"right_shaft_sign",1);
        assertThrows(IllegalArgumentException.class,()->SimConfig.parse(tmp,Map.of("drive",drive,"drive_geometry",Map.of("wheel_radius_m",.05,"track_width_m",.4,"wheelbase_m",.5))));
    }
    @Test void measuredProfileRequiresNewReviewUsesNativeWeightAndRoundTripsPortableBundle()throws Exception{
        var c=controller();var p=c.session.profile("robot");FieldPackage.map(p.get("runtime")).put("calibration_data",Map.of("battery",Map.of("v_internal",12.7,"r_battery",.15),"motors",Map.of()));c.update("robot",p);c.compile("robot");ready(c);
        Path saved=c.session.modelPath("robot");String original=Files.readString(saved);c.session.genericField();var scene=c.newScene();scene.put("robot_start_xyz_m",List.of(-.4,-.4,.08));c.session.scene(c.saveScene(scene,false),true);assertTrue(c.session.sceneReady());c.session.data.put("opmodes",List.of("Old code"));
        c.measurements(new RobotMeasurements.Entry(9.,90*RobotMeasurements.LengthUnit.MM.meters,40*RobotMeasurements.LengthUnit.CM.meters,.5));
        assertFalse(c.session.ready("robot"));assertFalse(c.session.sceneReady());assertNull(c.session.data.get("opmodes"));assertEquals(original,Files.readString(saved));
        try(var built=new ModelValidation.Prepared(c.session.modelPath("robot"),new DesktopAssetManager(true))){assertEquals(9,built.space.getRigidBodyList().stream().mapToDouble(b->b.getMass()).sum(),.0001);}
        ready(c);Path bundle=tmp.resolve("measurements.zip");c.models.run("export",c.session.modelPath("robot").toString(),bundle.toString());
        var imported=new GuidedSetupController(tmp.resolve("other-library"),null);imported.session.start("robot",null);imported.load("robot","bundle",bundle,null);assertTrue(imported.session.ready("robot"));
        var config=SimConfig.parse(tmp,Map.of("robot_model_profile",imported.session.modelPath("robot").toString(),"model_settings","profile"));
        assertEquals(9,config.totalMassKg);assertEquals(.045,config.driveGeometry.wheelRadiusM(),1e-9);assertEquals(.4,config.driveGeometry.trackWidthM());assertEquals(.5,config.driveGeometry.wheelbaseM());
        assertEquals(12.7,CalibrationProfile.load(Path.of(config.calibration)).vInternal);assertEquals(c.session.profile("robot").get("provenance"),imported.session.profile("robot").get("provenance"));
        var resumed=new GuidedSetupController(tmp.resolve("library"),null);resumed.resume(c.session.file);assertTrue(resumed.session.ready("robot"));
    }
    @Test void mecanumGeometryKeepsCadInferenceAndMeasuredOverrideChangesKinematics()throws Exception{
        var xml=new StringBuilder("<robot name=\"four wheels\"><link name=\"base\"/>");
        double[] radii={.04,.045,.05,.055};
        for(int i=0;i<4;i++){
            xml.append("<link name=\"w").append(i).append("\"><inertial><mass value=\".1\"/><inertia ixx=\".01\" iyy=\".01\" izz=\".01\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial><collision><geometry><cylinder radius=\"").append(radii[i]).append("\" length=\".02\"/></geometry></collision></link>");
            xml.append("<joint name=\"j").append(i).append("\" type=\"continuous\"><parent link=\"base\"/><child link=\"w").append(i).append("\"/><origin xyz=\"").append(i<2?.2:-.2).append(' ').append(i%2==0?.16:-.16).append(" 0\"/><axis xyz=\"0 1 0\"/></joint>");
            xml.append("<transmission name=\"t").append(i).append("\"><joint name=\"j").append(i).append("\"/><actuator name=\"").append(DriveGeometry.MOTORS.get(i)).append("\"><mechanicalReduction>1</mechanicalReduction></actuator></transmission>");
        }
        Path file=tmp.resolve("four.urdf");Files.writeString(file,xml.append("</robot>").toString());var robot=RobotUrdf.parse(file);
        var cad=DriveGeometry.resolve(robot,null);assertTrue(cad.positionsFromCad());assertEquals(.32,cad.trackWidthM());assertEquals(.4,cad.wheelbaseM());assertEquals(List.of(.04,.045,.05,.055),cad.wheelRadii());
        var measured=DriveGeometry.resolve(robot,new DriveGeometry(.06,.5,.6));assertEquals(List.of(.06,.06,.06,.06),measured.wheelRadii());
        var k=new physics.MecanumKinematics(measured.trackWidthM(),measured.wheelbaseM(),2.);
        var v=k.forwardFromWheelSpeeds(-.6,.6,-.6,.6);assertEquals(1.2/1.1,v.omega,1e-9);
        var fallback=DriveGeometry.resolve(null,null);assertEquals(.30,fallback.trackWidthM());assertEquals(.35,fallback.wheelbaseM());assertEquals(List.of(.048,.048,.048,.048),fallback.wheelRadii());
    }
    @Test void guideIdentifiesPassiveChainFollowersWithoutRequestingAnotherMotor()throws Exception{
        var c=controller();String xml="<robot name=\"chain\"><link name=\"base\"><visual><geometry><box size=\".3 .2 .1\"/></geometry></visual></link>";
        for(String name:List.of("powered","follower"))xml+="<link name=\""+name+"\"><visual><geometry><cylinder radius=\".02\" length=\".04\"/></geometry></visual></link><joint name=\""+name+"_joint\" type=\"continuous\"><parent link=\"base\"/><child link=\""+name+"\"/><axis xyz=\"0 1 0\"/>"+(name.equals("follower")?"<mimic joint=\"powered_joint\" multiplier=\"1\"/>":"")+"</joint>";
        xml+="<transmission name=\"tx\"><joint name=\"powered_joint\"/><actuator name=\"intake\"><mechanicalReduction>1</mechanicalReduction></actuator></transmission></robot>";
        Path zip=tmp.resolve("chain.zip");try(var z=new ZipOutputStream(Files.newOutputStream(zip))){z.putNextEntry(new ZipEntry("model.urdf"));z.write(xml.getBytes(java.nio.charset.StandardCharsets.UTF_8));z.closeEntry();}
        c.load("robot","cad",zip,null);var snapshot=RobotMeasurementGuide.read(c);
        var follower=snapshot.bindings().stream().filter(row->row.get(0).equals("follower")).findFirst().orElseThrow();
        assertEquals("Passive follower",follower.get(2));assertEquals("Coupled to powered_joint",follower.get(4));
        assertTrue(snapshot.bindings().stream().anyMatch(row->row.get(2).equals("intake")));
        Path project=tmp.resolve("missing-hardware");Files.createDirectories(project);ProfileIO.save(project.resolve("sim.config"),Map.of("robotConfig","missing.xml"));c.session.start("robot",project);
        var deferred=RobotMeasurementGuide.read(c);assertTrue(deferred.motors().contains("Manual measurements can still be saved"));assertTrue(deferred.bindings().stream().anyMatch(row->row.get(4).equals("Hardware check unavailable")));
    }
}
