package simrunner;

import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.math.Vector3f;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.scene.*;
import java.nio.file.*;
import java.util.*;
import simcore.*;
import static org.junit.jupiter.api.Assertions.*;

class PhysicsDiagnosticsTest {
    @TempDir Path tmp;
    static final float DT=1f/120;
    static final DesktopAssetManager ASSETS=new DesktopAssetManager(true);
    @BeforeAll static void natives(){com.jme3.system.NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    GuidedSetupController model(boolean mechanisms)throws Exception{return SyntheticRobots.importRobot(tmp,true,mechanisms,false,SyntheticRobots.Dimensions.standard());}
    PhysicsDiagnostics source(RobotMotionDemo d){return new PhysicsDiagnostics(d.world,d.scene,d.robot,d.setup.hardware(),d.setup.plan().driveNames);}
    @Test void headlessDemoExportsDiagnosticsWithoutChangingThePreparedModel()throws Exception {
        var c=model(false);Path profile=c.session.modelPath("robot"),report=tmp.resolve("session/demo.json");String original=Files.readString(profile);
        RobotMotionDemoApp.main(new String[]{profile.toString(),"-",report.toString(),"--headless","--diagnostics"});
        var data=MiniJson.parseObject(Files.readString(report));var diagnostics=FieldPackage.map(data.get("diagnostics"));
        assertEquals(1,((Number)diagnostics.get("schema_version")).intValue());assertEquals("N",FieldPackage.map(diagnostics.get("units")).get("force"));
        assertEquals(2,FieldPackage.maps(diagnostics.get("wheels")).size());assertFalse(FieldPackage.maps(data.get("observations")).isEmpty());assertEquals(original,Files.readString(profile));
        assertThrows(IllegalArgumentException.class,()->RobotMotionDemoApp.main(new String[]{profile.toString(),"-",report.toString(),"--headless","--screenshot",tmp.resolve("demo.png").toString()}));
    }
    @Test void supportedContactsShowLoadsNormalsAndExplicitlyUnavailableAggregateSlip()throws Exception {
        var c=model(false);try(var d=new RobotMotionDemo(c.motionSetup(),ASSETS,"drive/forward")) {
            SimulatorValidation.advance(d,.6,DT);var s=source(d).capture();assertEquals(2,s.wheels().size());assertTrue(s.wheels().stream().allMatch(PhysicsDiagnostics.Wheel::supported));
            double load=s.wheels().stream().mapToDouble(PhysicsDiagnostics.Wheel::loadN).sum();assertEquals(3*9.81,load,1);
            assertFalse(s.contacts().isEmpty());assertTrue(s.contacts().stream().allMatch(p->p.normal().y()>.7&&p.loadN()>=0));assertTrue(s.contacts().stream().anyMatch(p->p.loadN()>1));
            assertTrue(s.wheels().stream().allMatch(w->w.slipMps()==null&&w.forceN()==null));assertFalse(s.tireModel());
            var data=MiniJson.parseObject(ProfileIO.json(PhysicsDiagnostics.data(s)));assertEquals(null,FieldPackage.maps(data.get("wheels")).get(0).get("slip_mps"));assertTrue(PhysicsDiagnosticsOverlay.summary(s).contains("unavailable"));
            assertThrows(UnsupportedOperationException.class,()->s.notes().clear());
        }
    }
    @Test void missingSupportBellyScrapingAndZeroGripHaveDistinctObservedDiagnoses()throws Exception {
        var c=model(false);try(var d=new RobotMotionDemo(c.motionSetup(),ASSETS,"drive/forward")) {
            SimulatorValidation.floor(d).setFriction(0);SimulatorValidation.advance(d,.6,DT);assertTrue(source(d).capture().notes().stream().anyMatch(n->n.contains("grip is zero")));
            d.space.setGravity(Vector3f.ZERO);var b=d.world.chassisBody();b.setPhysicsLocation(b.getPhysicsLocation().add(0,2,0));d.space.update(DT,0);
            var air=source(d).capture();assertTrue(air.wheels().stream().noneMatch(PhysicsDiagnostics.Wheel::supported));assertTrue(air.notes().stream().anyMatch(n->n.contains("No native wheel support")));
        }
        var p=c.session.profile("robot");var settings=FieldPackage.map(FieldPackage.map(FieldPackage.map(p.get("entities")).get("base")).get("settings"));settings.put("collision_strategy","box");settings.put("box_size_m",List.of(.3,.2,.16));c.update("robot",p);c.compile("robot");
        try(var d=new RobotMotionDemo(c.motionSetup(),ASSETS,"drive/forward")){SimulatorValidation.advance(d,.6,DT);assertTrue(source(d).capture().notes().stream().anyMatch(n->n.contains("scrapes the floor")));}
    }
    @Test void driveBlockingRequiresBothCommandAndObstacleEvidence()throws Exception {
        var c=model(false);try(var d=new RobotMotionDemo(c.motionSetup(),ASSETS,"drive/forward")) {
            var wall=new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(.02f,1,2)),0);wall.setPhysicsLocation(new Vector3f(.35f,.5f,0));d.space.add(wall);SimulatorValidation.advance(d,.6,DT);
            d.world.driveChassis(new physics.MecanumKinematics.ChassisVelocity(2,0,0),DT);d.setup.hardware().get(SimDcMotorEx.class,"leftDrive").setPower(-.7);d.setup.hardware().get(SimDcMotorEx.class,"rightDrive").setPower(.7);SimulatorValidation.advance(d,2,DT);
            var s=source(d).capture();assertTrue(s.contacts().stream().anyMatch(p->Math.abs(p.normal().y())<.7&&p.loadN()>.01));assertTrue(s.notes().stream().anyMatch(n->n.contains("possible blocking")));
            for(var motor:d.setup.hardware().getAll(SimDcMotorEx.class))motor.setPower(0);assertTrue(source(d).capture().notes().stream().noneMatch(n->n.contains("Drive effort")));
        }
    }
    @Test void tireSlipAndForceAreRealSolverValuesWithCorrectUnitsAndCaps()throws Exception {
        var c=model(false);var p=c.session.profile("robot");var spec=Map.of("static_mu",.9,"sliding_mu",.6,"lateral_scale",1.,"stiffness_n_per_mps",100.,"transition_mps",.15);
        FieldPackage.map(p.get("runtime")).put("tires",Map.of("traction",spec,"omni",spec,"omni_joints",List.of(),"reflected_motor_inertia_kg_m2",.0015,"contact_tolerance_m",.004));c.update("robot",p);c.compile("robot");
        try(var d=new RobotMotionDemo(c.motionSetup(),ASSETS,"drive/forward")) {
            SimulatorValidation.floor(d).setFriction(0);SimulatorValidation.advance(d,.6,DT);var motors=d.setup.hardware().getAll(SimDcMotorEx.class);d.setup.hardware().get(SimDcMotorEx.class,"leftDrive").setPower(-.7);d.setup.hardware().get(SimDcMotorEx.class,"rightDrive").setPower(.7);
            for(int i=0;i<240;i++){for(var m:motors)m.integrate(12,DT,i*8);d.space.update(DT,0);}
            var s=source(d).capture();assertTrue(s.tireModel());for(var w:s.wheels()){assertTrue(w.supported());assertNotNull(w.slipMps());assertTrue(w.slipMps()>.5);assertEquals(0,w.forceN(),.00001);assertEquals(0,w.gripN(),.00001);assertTrue(w.loadN()>1);}
            assertTrue(s.motors().stream().allMatch(m->Math.abs(m.speedRadS())>10));assertTrue(PhysicsDiagnostics.data(s).containsKey("units"));
        }
    }
    @Test void cachedMechanismTelemetryShowsUnitsLimitsAndContactBlockedEffort()throws Exception {
        var c=model(true);try(var d=new RobotMotionDemo(c.motionSetup(),ASSETS,"joint/slide_joint")) {
            d.space.setGravity(Vector3f.ZERO);d.world.chassisBody().setMass(0);var slide=d.robot.bodyForLink("slide");
            var wall=new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(.005f,1,1)),0);wall.setPhysicsLocation(slide.getPhysicsLocation().add(.04f,0,0));d.space.add(wall);
            var motor=d.setup.hardware().get(SimDcMotorEx.class,"slideMotor");motor.setPower(.2);
            for(int i=0;i<240;i++){motor.integrate(12,DT,i*8);d.space.update(DT,0);}
            var s=source(d).capture();var j=s.joints().stream().filter(a->a.name().equals("slide_joint")).findFirst().orElseThrow();assertEquals("m",j.unit());assertEquals(0,j.lower());assertEquals(.08,j.upper());assertTrue(j.position()<.02);assertTrue(j.effort()>.01);
            assertTrue(s.notes().stream().anyMatch(n->n.contains("slide_joint")&&n.contains("possible blocking/load")));assertTrue(s.joints().stream().anyMatch(a->a.unit().equals("rad")));
        }
    }
    @Test void pollingAndOverlayTogglesCannotMutateNativeStateOrChangeDemoMotion()throws Exception {
        var c=model(true);var runs=new ArrayList<List<Map<String,Object>>>();
        for(boolean enabled:new boolean[]{false,true})try(var d=new RobotMotionDemo(c.motionSetup(),ASSETS)) {
            var diagnostics=source(d);var world=new Node();var gui=new Node();var overlay=new PhysicsDiagnosticsOverlay(world,gui,ASSETS,ASSETS.loadFont("Interface/Fonts/Default.fnt"),diagnostics::capture);overlay.setVisible(enabled);
            int ticks=0;while(!d.finished()&&ticks++<30000){d.tick();if(enabled){var before=d.world.chassisBody().getPhysicsLocation();var omega=d.setup.hardware().get(SimDcMotorEx.class,"armMotor").getPhysicalShaftRadians();int bodies=d.space.countRigidBodies();diagnostics.capture();assertEquals(before,d.world.chassisBody().getPhysicsLocation());assertEquals(omega,d.setup.hardware().get(SimDcMotorEx.class,"armMotor").getPhysicalShaftRadians());assertEquals(bodies,d.space.countRigidBodies());overlay.update(RobotMotionDemo.DT,1280,800);}}
            assertTrue(d.finished());assertEquals("",d.failure);runs.add(List.copyOf(d.observations));overlay.dispose();assertEquals(0,world.getQuantity());assertEquals(0,gui.getQuantity());
        }
        assertEquals(runs.get(0).size(),runs.get(1).size());for(int i=0;i<runs.get(0).size();i++){var a=runs.get(0).get(i);var b=runs.get(1).get(i);assertEquals(a.get("outcome"),b.get("outcome"));for(String key:List.of("forward_m","left_m","yaw_rad","start","end","peak_travel"))if(a.containsKey(key))assertEquals(FieldPackage.num(a,key),FieldPackage.num(b,key),key.equals("forward_m")||key.equals("left_m")?.005:.03,key);}
    }
    @Test void pooledOverlayIsThrottledHiddenAndRemovableAndSnapshotExportIsReadOnly()throws Exception {
        var c=model(true);try(var d=new RobotMotionDemo(c.motionSetup(),ASSETS)) {
            SimulatorValidation.advance(d,.6,DT);var source=source(d);int[] polls={0};var root=new Node();var gui=new Node();var overlay=new PhysicsDiagnosticsOverlay(root,gui,ASSETS,ASSETS.loadFont("Interface/Fonts/Default.fnt"),()->{polls[0]++;return source.capture();});
            overlay.update(1,1280,800);assertEquals(0,polls[0]);overlay.setVisible(true);overlay.update(0,1280,800);assertEquals(1,polls[0]);assertTrue(overlay.vectorCount()>0);int count=overlay.root.getQuantity();overlay.update(.04f,1280,800);assertEquals(1,polls[0]);overlay.update(.07f,1280,800);assertEquals(2,polls[0]);assertEquals(count,overlay.root.getQuantity());assertTrue(count<=PhysicsDiagnostics.MAX_CONTACTS*2+PhysicsDiagnostics.MAX_ITEMS*2+1);
            assertTrue(overlay.text.getLineCount()*overlay.text.getLineHeight()<656,"Panel overflow: "+overlay.text.getLineCount());
            String original=Files.readString(c.session.modelPath("robot"));Path saved=overlay.save(tmp.resolve("snapshots"));assertEquals(original,Files.readString(c.session.modelPath("robot")));assertEquals(1,((Number)MiniJson.parseObject(Files.readString(saved)).get("schema_version")).intValue());
            overlay.setVisible(false);assertEquals(Spatial.CullHint.Always,overlay.root.getCullHint());assertEquals(Spatial.CullHint.Always,overlay.panel.getCullHint());int before=polls[0];overlay.update(1,1280,800);assertEquals(before,polls[0]);overlay.dispose();assertEquals(0,root.getQuantity());assertEquals(0,gui.getQuantity());
            assertEquals(.35,PhysicsDiagnosticsOverlay.scaledForce(new Vector3f(100,0,0)).length(),.0001);
        }
    }
    @Test void sharedKeyboardBindingsToggleSaveAndFollowReplayedOverlay()throws Exception {
        var c=model(false);try(var d=new RobotMotionDemo(c.motionSetup(),ASSETS)) {
            var root=new Node();var gui=new Node();var diagnostics=source(d);var font=ASSETS.loadFont("Interface/Fonts/Default.fnt");
            var current=new PhysicsDiagnosticsOverlay[]{new PhysicsDiagnosticsOverlay(root,gui,ASSETS,font,diagnostics::capture)};
            var mouse=new com.jme3.input.dummy.DummyMouseInput();var keyboard=new QueuedKeyInput();mouse.initialize();keyboard.initialize();
            var input=new com.jme3.input.InputManager(mouse,keyboard,null,null);var errors=new ArrayList<Exception>();
            PhysicsDiagnosticControls.bind(input,()->current[0],tmp.resolve("keyboard-export"),errors::add);
            key(input,keyboard,com.jme3.input.KeyInput.KEY_D,'d');assertTrue(current[0].visible());key(input,keyboard,com.jme3.input.KeyInput.KEY_D,'d');assertFalse(current[0].visible());
            key(input,keyboard,com.jme3.input.KeyInput.KEY_P,'p');try(var files=Files.list(tmp.resolve("keyboard-export"))){assertEquals(1,files.count());}assertTrue(errors.isEmpty());
            current[0].dispose();current[0]=new PhysicsDiagnosticsOverlay(root,gui,ASSETS,font,diagnostics::capture);key(input,keyboard,com.jme3.input.KeyInput.KEY_D,'d');assertTrue(current[0].visible());current[0].dispose();
        }
    }
    private static final class QueuedKeyInput extends com.jme3.input.dummy.DummyKeyInput {
        com.jme3.input.RawInputListener listener;
        com.jme3.input.event.KeyInputEvent pending;
        public void setInputListener(com.jme3.input.RawInputListener value){listener=value;}
        public void update(){super.update();if(pending!=null){pending.setTime(getInputTimeNanos());listener.onKeyEvent(pending);pending=null;}}
    }
    private static void key(com.jme3.input.InputManager input,QueuedKeyInput keyboard,int code,char character) {
        keyboard.pending=new com.jme3.input.event.KeyInputEvent(code,character,true,false);input.update(.01f);
        keyboard.pending=new com.jme3.input.event.KeyInputEvent(code,character,false,false);input.update(.01f);
    }
    @Test void legacySupportMissingBindingsAndSamplingLimitsAreExplicit()throws Exception {
        var c=model(false);var p=c.session.profile("robot");FieldPackage.map(FieldPackage.map(p.get("runtime")).get("drive_contacts")).put("enabled",false);c.update("robot",p);c.compile("robot");
        try(var d=new RobotMotionDemo(c.motionSetup(),ASSETS)) {
            for(int i=0;i<40;i++)d.setup.hardware().register("extra"+i,new SimDcMotorEx("extra"+i,new physics.MotorSpec("test",1,2,9.2,30,12,500)));
            var s=new PhysicsDiagnostics(d.world,d.scene,d.robot,d.setup.hardware(),Set.of("missingDrive")).capture();assertTrue(s.truncated());assertEquals(32,s.motors().size());assertTrue(s.notes().stream().anyMatch(n->n.contains("legacy proxy")));assertTrue(s.notes().stream().anyMatch(n->n.contains("missingDrive")));assertTrue(s.notes().stream().anyMatch(n->n.contains("sampling limit")));
        }
    }
}
