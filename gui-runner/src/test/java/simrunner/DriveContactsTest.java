package simrunner;

import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import static org.junit.jupiter.api.Assertions.*;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.collision.*;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import java.nio.file.Path;
import java.util.*;
import physics.MecanumKinematics;

class DriveContactsTest {
    @TempDir Path tmp;
    @BeforeAll static void nativePhysics(){NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    GuidedSetupController model()throws Exception{var f=new RobotMotionDemoTest();f.tmp=tmp;return f.model(true,false);}
    void complete(RobotMotionDemo demo){for(int i=0;i<10000&&!demo.finished();i++)demo.tick();assertTrue(demo.finished());assertEquals("",demo.failure);}
    PhysicsRigidBody floor(RobotMotionDemo demo){return demo.space.getRigidBodyList().stream().filter(b->b.isStatic()&&b.getPhysicsLocation().y<0).findFirst().orElseThrow();}
    void advance(RobotMotionDemo demo,double seconds,float dt){for(int i=0;i<Math.round(seconds/dt);i++)demo.space.update(dt,0);}
    @Test void defaultFrictionLowPowerMovesWithWheelSupportAndLegacyProxyStillReproducesTheStall()throws Exception {
        var c=model();assertEquals(.6,FieldPackage.num(FieldPackage.map(c.session.profile("robot").get("parameters")),"friction"));
        try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")) {
            complete(demo);var r=demo.observations.get(0);assertEquals("movement observed",r.get("outcome"),r.toString());assertTrue(FieldPackage.num(r,"forward_m")>.03,r.toString());assertEquals(2,((Number)r.get("supported_wheels")).intValue());assertEquals(0,((Number)r.get("scraping_contacts")).intValue());assertTrue(RobotMotionReview.contactSummary(r).contains("Supported wheels: 2"));assertEquals(.6,demo.world.chassisBody().getFriction(),.0001);System.out.println("[DRIVE CONTACT BASELINE] "+r);
        }
        var p=c.session.profile("robot");FieldPackage.map(p.get("runtime")).remove("drive_contacts");c.update("robot",p);c.compile("robot");
        try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")){complete(demo);assertEquals("no clear movement",demo.observations.get(0).get("outcome"));System.out.println("[LEGACY DEFAULT FRICTION] "+demo.observations.get(0));}
    }
    @Test void wheelsAboveGroundDoNotInventTractionOrCancelExternalPushes()throws Exception {
        var c=model();try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")) {
            demo.space.setGravity(Vector3f.ZERO);var body=demo.world.chassisBody();body.setPhysicsLocation(body.getPhysicsLocation().add(0,2,0));
            demo.world.driveChassis(new MecanumKinematics.ChassisVelocity(2,0,1),1f/120);advance(demo,1,1f/120);
            assertEquals(0,body.getLinearVelocity().x,.0001);assertEquals(0,body.getAngularVelocity().y,.0001);assertTrue(demo.world.driveContacts().snapshot().wheels().isEmpty());
            body.applyCentralImpulse(new Vector3f(body.getMass(),0,0));float start=body.getPhysicsLocation().x;advance(demo,.5,1f/120);assertEquals(1,body.getLinearVelocity().x,.001);assertTrue(body.getPhysicsLocation().x-start>.45);
            demo.space.setGravity(new Vector3f(0,-9.81f,0));advance(demo,.15,1f/120);assertTrue(body.getLinearVelocity().y<-1);
        }
    }
    @Test void bellyGroundingKeepsNativeFrictionAndReportsMissingWheelSupport()throws Exception {
        var c=model();var p=c.session.profile("robot");var base=FieldPackage.map(FieldPackage.map(FieldPackage.map(p.get("entities")).get("base")).get("settings"));base.put("collision_strategy","box");base.put("box_size_m",List.of(.3,.2,.16));c.update("robot",p);c.compile("robot");
        try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")) {
            complete(demo);var r=demo.observations.get(0);assertEquals("no clear movement",r.get("outcome"));assertEquals(0,((Number)r.get("supported_wheels")).intValue());assertTrue(((Number)r.get("scraping_contacts")).intValue()>0);assertTrue(r.get("detail").toString().contains("chassis clearance"));
            assertTrue(combinedFrictions(demo,floor(demo)).stream().anyMatch(mu->mu>.3));
        }
    }
    List<Float> combinedFrictions(RobotMotionDemo demo,PhysicsRigidBody other) {
        var values=new ArrayList<Float>();long body=demo.world.chassisBody().nativeId(),ground=other.nativeId();
        for(long manifold:demo.space.listManifoldIds()){long a=PersistentManifolds.getBodyAId(manifold),b=PersistentManifolds.getBodyBId(manifold);if(a==body&&b==ground||a==ground&&b==body)for(long point:PersistentManifolds.listPointIds(manifold))values.add(ManifoldPoints.getCombinedFriction(point));}return values;
    }
    @ParameterizedTest @ValueSource(booleans={false,true})
    void wheelFloorTangentialOwnershipPreservesWallFrictionAndCollisionBlocking(boolean wheelOnly)throws Exception {
        var c=model();try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")) {
            var shape=new BoxCollisionShape(new Vector3f(.02f,1,wheelOnly?.008f:2));shape.setMargin(.001f);
            var wall=new PhysicsRigidBody(shape,0);wall.setPhysicsLocation(new Vector3f(.35f,.5f,wheelOnly?-.13f:0));wall.setFriction(1);demo.space.add(wall);
            if(wheelOnly){var second=new PhysicsRigidBody(shape,0);second.setPhysicsLocation(new Vector3f(.35f,.5f,.13f));second.setFriction(1);demo.space.add(second);}
            advance(demo,.4,1f/120);demo.world.driveChassis(new MecanumKinematics.ChassisVelocity(2,0,0),1f/120);
            if(wheelOnly) {
                // Narrow obstacles intentionally miss the chassis and strike only wheel children.
                // Inspect first contact: a robot can eventually steer around separate posts.
                for(int i=0;i<480&&combinedFrictions(demo,wall).isEmpty();i++)advance(demo,1f/120,1f/120);
            } else advance(demo,4,1f/120);
            assertTrue(demo.world.robotPointWorld(Vector3f.ZERO).x<.19,"Must stop behind wall: "+demo.world.getChassisPosition());assertEquals(2,demo.world.driveContacts().snapshot().wheels().size());
            assertTrue(combinedFrictions(demo,wall).stream().anyMatch(mu->mu>.5),"Wall friction must stay native");assertTrue(combinedFrictions(demo,floor(demo)).stream().allMatch(mu->mu==0),"Only wheel/floor contacts are owned");
            demo.world.driveChassis(new MecanumKinematics.ChassisVelocity(0,0,0),1f/120);demo.world.chassisBody().applyCentralImpulse(new Vector3f(-3,0,0));float before=demo.world.getChassisPosition().x;advance(demo,.1,1f/120);assertTrue(demo.world.getChassisPosition().x<before-.01,"Finite braking must allow push displacement");
        }
    }
    @Test void rollingResistanceDissipatesSupportedCoastingEnergy()throws Exception {
        double[] speeds=new double[2];
        for(int i=0;i<2;i++) {
            var c=model();var p=c.session.profile("robot");FieldPackage.map(FieldPackage.map(p.get("runtime")).get("drive_contacts")).put("rolling_resistance_coefficient",i==0?0.:.05);c.update("robot",p);c.compile("robot");
            try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")) {
                advance(demo,.4,1f/120);demo.world.configureDrive(1000,10,1000,10);
                demo.world.chassisBody().setLinearVelocity(new Vector3f(1,0,0));advance(demo,.5,1f/120);speeds[i]=demo.world.chassisBody().getLinearVelocity().x;
            }
        }
        assertTrue(speeds[0]>.99);assertTrue(speeds[1]>0&&speeds[1]<speeds[0]-.2,Arrays.toString(speeds));
    }
    @ParameterizedTest @ValueSource(booleans={false,true})
    void nativeTiresRespectFloorGripAndKeepEqualShaftReaction(boolean wheelOverride)throws Exception {
        var c=model();var p=c.session.profile("robot");var spec=Map.of("static_mu",.9,"sliding_mu",.8,"lateral_scale",1.,"stiffness_n_per_mps",100.,"transition_mps",.15);
        FieldPackage.map(p.get("runtime")).put("tires",Map.of("traction",spec,"omni",spec,"omni_joints",List.of(),"reflected_motor_inertia_kg_m2",.0015,"contact_tolerance_m",.004));c.update("robot",p);c.compile("robot");
        double[] travel=new double[2],shaftSpeed=new double[2];
        for(int i=0;i<2;i++) {
            if(wheelOverride){c.wheelGrip("wheel_joint0",i==0?.6:0.);c.wheelGrip("wheel_joint1",i==0?.6:0.);}
            try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")) {
            floor(demo).setFriction(wheelOverride?.6f:i==0?.6f:0);
            while(!demo.finished()) {demo.tick();shaftSpeed[i]=Math.max(shaftSpeed[i],demo.world.tireDrive().states().stream().mapToDouble(w->Math.abs(w.surfaceMps())).max().orElse(0));}
            assertEquals("",demo.failure);var r=demo.observations.get(0);travel[i]=FieldPackage.num(r,"forward_m");
            assertTrue(demo.world.tireDrive().states().stream().allMatch(TireDrive.State::supported));
            assertEquals(.6,demo.world.chassisBody().getFriction(),.0001);
            if(i==1)assertTrue(demo.world.tireDrive().states().stream().allMatch(w->w.forceN()==0));
        }}
        System.out.println("[NATIVE TIRES] wheel override="+wheelOverride+" grip/zero-grip travel="+Arrays.toString(travel)+" peak shaft speed="+Arrays.toString(shaftSpeed));
        assertTrue(travel[0]>.005&&travel[0]>travel[1]*10,Arrays.toString(travel));assertEquals(0,travel[1],.001);assertTrue(shaftSpeed[1]>shaftSpeed[0],Arrays.toString(shaftSpeed));
    }
    @Test void decomposedMeshWheelsRetainContactIdentityUnderBodyRotation()throws Exception {
        var f=new RobotMotionDemoTest();f.tmp=tmp;var c=model();String xml=f.fixture(true,false).replace("<cylinder radius=\".045\" length=\".02\"/>","<mesh filename=\"wheel.stl\"/>");
        Path zip=tmp.resolve("mesh.zip");try(var out=new java.util.zip.ZipOutputStream(java.nio.file.Files.newOutputStream(zip))) {
            out.putNextEntry(new java.util.zip.ZipEntry("model.urdf"));out.write(xml.getBytes(java.nio.charset.StandardCharsets.UTF_8));out.closeEntry();
            out.putNextEntry(new java.util.zip.ZipEntry("wheel.stl"));out.write(wheelStl().getBytes(java.nio.charset.StandardCharsets.UTF_8));out.closeEntry();
        }
        c.load("robot","fresh",zip,null);var p=c.session.profile("robot");FieldPackage.map(p.get("runtime")).put("drive",Map.of("type","differential","left_motor","leftDrive","right_motor","rightDrive","wheel_radius_m",.045,"track_width_m",.26,"left_shaft_sign",-1,"right_shaft_sign",1));
        for(String n:List.of("wheel0","wheel1"))FieldPackage.map(FieldPackage.map(FieldPackage.map(p.get("entities")).get(n)).get("settings")).put("collision_strategy","mesh");c.update("robot",p);c.compile("robot");
        try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")) {
            var shape=(com.jme3.bullet.collision.shapes.CompoundCollisionShape)demo.world.chassisBody().getCollisionShape();
            assertTrue(Arrays.stream(shape.listChildren()).noneMatch(child->child.getShape() instanceof com.jme3.bullet.collision.shapes.CompoundCollisionShape));
            assertEquals(3,demo.world.chassisBody().getMass(),.0001,"Geometry must not duplicate welded mass");
            var body=demo.world.chassisBody();body.setPhysicsRotation(new com.jme3.math.Quaternion().fromAngleAxis(.7f,Vector3f.UNIT_Y).mult(body.getPhysicsRotation()));
            advance(demo,.5,1f/120);var start=body.getPhysicsLocation();var forward=demo.world.getChassisRotation().mult(Vector3f.UNIT_X);
            demo.world.driveChassis(new MecanumKinematics.ChassisVelocity(1,0,0),1f/120);advance(demo,.3,1f/120);
            assertEquals(2,demo.world.driveContacts().snapshot().wheels().size());assertTrue(body.getPhysicsLocation().subtract(start).dot(forward)>.04);
            assertTrue(combinedFrictions(demo,floor(demo)).stream().allMatch(mu->mu==0));
        }
    }
    String wheelStl() {
        double[][] v={{-.045,-.045,-.01},{.045,-.045,-.01},{.045,.045,-.01},{-.045,.045,-.01},{-.045,-.045,.01},{.045,-.045,.01},{.045,.045,.01},{-.045,.045,.01}};
        int[][] faces={{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},{1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};var stl=new StringBuilder("solid wheel\n");
        for(var face:faces){stl.append("facet normal 0 0 0\nouter loop\n");for(int i:face)stl.append("vertex ").append(v[i][0]).append(" ").append(v[i][1]).append(" ").append(v[i][2]).append("\n");stl.append("endloop\nendfacet\n");}return stl.append("endsolid wheel\n").toString();
    }
    @Test void missingWheelCollisionCannotPassNativeReview()throws Exception {
        var c=model();var p=c.session.profile("robot");FieldPackage.map(FieldPackage.map(FieldPackage.map(p.get("entities")).get("wheel0")).get("settings")).put("collision_strategy","none");c.update("robot",p);c.compile("robot");
        var error=assertThrows(IllegalArgumentException.class,()->new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward"));assertTrue(error.getMessage().contains("wheel_joint0"));assertTrue(error.getMessage().contains("collision geometry"));
    }
    @ParameterizedTest @ValueSource(floats={0.033333334f,0.008333334f,0.0020833334f})
    void tractionIsMaterialBoundedAndStableAtDifferentNativeTimesteps(float dt)throws Exception {
        var c=model();var p=c.session.profile("robot");FieldPackage.map(FieldPackage.map(p.get("runtime")).get("drive_contacts")).put("rolling_resistance_coefficient",0.);c.update("robot",p);c.compile("robot");
        try(var demo=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true),"drive/forward")) {
            floor(demo).setFriction(.01f);advance(demo,.5,dt);float start=demo.world.getChassisPosition().x;
            demo.world.driveChassis(new MecanumKinematics.ChassisVelocity(5,0,0),dt);advance(demo,1,dt);float speed=demo.world.chassisBody().getLinearVelocity().x;
            assertTrue(speed>0.025&&speed<.085,"Grip bounds acceleration, speed="+speed+" dt="+dt);assertTrue(demo.world.getChassisPosition().x-start<.06);assertTrue(Float.isFinite(demo.world.getChassisPosition().x));
            floor(demo).setFriction(0);demo.world.chassisBody().setLinearVelocity(Vector3f.ZERO);advance(demo,.25,dt);assertEquals(0,demo.world.chassisBody().getLinearVelocity().x,.001);
        }
    }
}
