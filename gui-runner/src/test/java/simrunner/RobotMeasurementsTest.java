package simrunner;

import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
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
