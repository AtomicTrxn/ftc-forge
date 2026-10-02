package simrunner;

import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import java.nio.file.*;
import java.util.*;
import physics.MecanumKinematics;

class WheelGripTest {
    @TempDir Path tmp;
    @BeforeAll static void nativePhysics(){NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    DriveContactsTest fixture(){var f=new DriveContactsTest();f.tmp=tmp;return f;}
    void chassisFriction(GuidedSetupController c,double mu)throws Exception{var p=c.session.profile("robot");FieldPackage.map(p.get("parameters")).put("friction",mu);c.update("robot",p);c.compile("robot");}
    @Test void differentWheelsUseTheirOwnGripBudgetsWithoutChangingBodyMaterial()throws Exception {
        var f=fixture();var c=f.model();chassisFriction(c,.02);c.wheelGrip("wheel_joint0",0.);c.wheelGrip("wheel_joint1",1.2);
        try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")) {
            f.advance(demo,.5,1f/120);var support=demo.world.driveContacts().snapshot();assertEquals(2,support.wheels().size());
            for(var wheel:support.wheels()){assertTrue(wheel.normalImpulse()>0);assertEquals(wheel.joint().equals("wheel_joint0")?0:1.2*.6,wheel.frictionImpulse()/wheel.normalImpulse(),.00001);}
            assertEquals(.02,demo.world.chassisBody().getFriction(),.00001);assertEquals(3,demo.world.chassisBody().getMass(),.0001);
            f.floor(demo).setFriction(0);f.advance(demo,.1,1f/120);assertEquals(0,demo.world.driveContacts().snapshot().frictionImpulse(),.0000001);
        }
    }
    @Test void rubberCanDriveASlipperyChassisAndZeroGripWheelsCannotDriveAGrippyChassis()throws Exception {
        var f=fixture();var c=f.model();double[] travel=new double[2];
        for(int i=0;i<2;i++) {
            chassisFriction(c,i==0?0.:.9);c.wheelGrip("wheel_joint0",i==0?.9:0.);c.wheelGrip("wheel_joint1",i==0?.9:0.);
            try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")){f.complete(demo);var row=demo.observations.get(0);travel[i]=FieldPackage.num(row,"forward_m");assertEquals(2,((Number)row.get("supported_wheels")).intValue());assertTrue(RobotMotionReview.contactSummary(row).contains("Simulated effective grip"));}
        }
        assertTrue(travel[0]>.03,Arrays.toString(travel));assertEquals(0,travel[1],.001);System.out.println("[WHEEL GRIP] rubber/slippery-body vs zero-rubber/grippy-body travel="+Arrays.toString(travel));
    }
    @Test void wheelObstacleFrictionUsesRubberWhileBellyKeepsItsOwnMaterial()throws Exception {
        var f=fixture();var c=f.model();chassisFriction(c,.15);c.wheelGrip("wheel_joint0",1.2);c.wheelGrip("wheel_joint1",1.2);
        try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")) {
            var shape=new BoxCollisionShape(new Vector3f(.02f,1,.008f));shape.setMargin(.001f);
            var post=new PhysicsRigidBody(shape,0);post.setPhysicsLocation(new Vector3f(.35f,.5f,-.13f));post.setFriction(.5f);demo.space.add(post);
            f.advance(demo,.5,1f/120);demo.world.driveChassis(new MecanumKinematics.ChassisVelocity(2,0,0),1f/120);
            for(int i=0;i<480&&f.combinedFrictions(demo,post).isEmpty();i++)f.advance(demo,1f/120,1f/120);
            assertFalse(f.combinedFrictions(demo,post).isEmpty());assertTrue(f.combinedFrictions(demo,post).stream().allMatch(mu->Math.abs(mu-.6)<.00001));assertEquals(.15,demo.world.chassisBody().getFriction(),.00001);
        }
        var p=c.session.profile("robot");var base=FieldPackage.map(FieldPackage.map(FieldPackage.map(p.get("entities")).get("base")).get("settings"));base.put("collision_strategy","box");base.put("box_size_m",List.of(.3,.2,.16));c.update("robot",p);c.compile("robot");
        try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")){f.advance(demo,.5,1f/120);assertTrue(demo.world.driveContacts().snapshot().wheels().isEmpty());assertTrue(f.combinedFrictions(demo,f.floor(demo)).stream().allMatch(mu->Math.abs(mu-.09)<.00001));assertFalse(f.combinedFrictions(demo,f.floor(demo)).isEmpty());}
    }
    @Test void guidedGripEditsInvalidateReviewAndRoundTripWithoutChangingSavedRevision()throws Exception {
        var f=fixture();var c=f.model();var validator=new GuidedSetupTest();validator.tmp=tmp;
        assertEquals(Set.of("wheel_joint0","wheel_joint1"),c.wheelGripChoices().keySet());validator.ready(c,"robot");Path saved=c.session.modelPath("robot");String before=Files.readString(saved),context=c.motionSetup().context();
        c.wheelGrip("wheel_joint0",.93);assertFalse(c.session.ready("robot"));assertNotEquals(context,c.motionSetup().context());assertNotEquals(saved,c.session.modelPath("robot"));assertEquals(before,Files.readString(saved));
        assertEquals("user supplied: configured drive wheel selection",FieldPackage.map(c.session.profile("robot").get("provenance")).get("runtime/drive_contacts/wheel_friction/0/joint"));
        c.wheelGrip("wheel_joint1",.41);var measured=c.session.profile("robot");FieldPackage.map(measured.get("provenance")).put("runtime/drive_contacts/wheel_friction/1/friction","measured manually: tread/field pair");c.update("robot",measured);c.compile("robot");validator.ready(c,"robot");Path bundle=tmp.resolve("grip.zip");c.models.run("export",c.session.modelPath("robot").toString(),bundle.toString());
        var restored=new GuidedSetupController(tmp.resolve("portable"),null);restored.session.start("robot",null);restored.load("robot","bundle",bundle,null);
        assertTrue(restored.session.ready("robot"));assertEquals(Map.of("wheel_joint0",.93,"wheel_joint1",.41),restored.motionSetup().config().driveContacts.wheelFriction());restored.wheelGrip("wheel_joint1",.41);assertTrue(restored.session.ready("robot"),"Unchanged grip must retain review and measured provenance");
        restored.wheelGrip("wheel_joint0",null);assertFalse(restored.session.ready("robot"));var cfg=restored.motionSetup().config().driveContacts;assertEquals(Map.of("wheel_joint1",.41),cfg.wheelFriction());assertEquals(.6,cfg.friction("wheel_joint0",.6));assertEquals(before,Files.readString(saved));
        assertEquals("measured manually: tread/field pair",FieldPackage.map(restored.session.profile("robot").get("provenance")).get("runtime/drive_contacts/wheel_friction/0/friction"));
        assertThrows(IllegalArgumentException.class,()->restored.wheelGrip("lift",.8));assertThrows(IllegalArgumentException.class,()->restored.wheelGrip("wheel_joint0",Double.NaN));
        restored.wheelSupport(false);assertThrows(IllegalArgumentException.class,()->restored.wheelGrip("wheel_joint1",.9));
    }
    @Test void invalidAndUnusedOverridesFailInsteadOfBeingSilentlyIgnored()throws Exception {
        var cfg=DriveContactConfig.defaults(true);
        for(Object bad:List.of("bad",List.of(Map.of("joint","x","friction",true)),List.of(Map.of("joint","x","friction",-1)),List.of(Map.of("joint","x","friction",2.1)),List.of(Map.of("joint","x","friction",Double.POSITIVE_INFINITY)),List.of(Map.of("joint","x","friction",.4),Map.of("joint","x","friction",.5)))){cfg.put("wheel_friction",bad);assertThrows(IllegalArgumentException.class,()->DriveContactConfig.parse(cfg));}
        var f=new RobotMotionDemoTest();f.tmp=tmp;var c=f.model(true,true);var p=c.session.profile("robot");FieldPackage.map(FieldPackage.map(p.get("runtime")).get("drive_contacts")).put("wheel_friction",List.of(Map.of("joint","arm_joint","friction",.7)));c.update("robot",p);c.compile("robot");
        assertTrue(assertThrows(IllegalArgumentException.class,()->new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true))).getMessage().contains("not a configured drive wheel"));
    }
}
