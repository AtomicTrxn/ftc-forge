package simrunner;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.math.*;
import com.jme3.scene.Node;
import com.jme3.system.NativeLibraryLoader;
import com.qualcomm.robotcore.hardware.*;
import simcore.*;
import physics.*;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.*;
import java.util.*;
import static org.junit.jupiter.api.Assertions.*;
class ContactModelsTest {
    @TempDir Path temp;
    @BeforeAll static void natives(){NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    record Result(double distance,double omega,double current,double maxSlip,int supported){}
    Result drive(double mu,boolean airborne,boolean wall,float dt) throws Exception {
        PhysicsSpace space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(airborne?Vector3f.ZERO:new Vector3f(0,-9.81f,0));
            var world=new PhysicsWorld(null,new Node(),space);
            world.buildChassis(new Node(),3,new Vector3f(0,airborne?1:.05f,0),new BoxCollisionShape(new Vector3f(.1f,.05f,.1f)));
            world.chassisBody().setEnableSleep(false);
            var floor=new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(5,.01f,5)),0);
            floor.setPhysicsLocation(new Vector3f(0,-.01f,0));space.add(floor);
            if(wall){var obstacle=new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(.03f,1,1)),0);obstacle.setPhysicsLocation(new Vector3f(.4f,0,0));space.add(obstacle);}
            var map=new HardwareMap();var left=new SimDcMotorEx("left",new MotorSpec("test",1,2,8,30,12,500));
            var right=new SimDcMotorEx("right",new MotorSpec("test",1,2,8,30,12,500));map.register("left",left);map.register("right",right);
            left.setDirection(DcMotor.Direction.REVERSE);left.setPower(.7);right.setPower(.7);
            Path file=temp.resolve("base.urdf");Files.writeString(file,"<robot name='fixture'><link name='base'><inertial><mass value='3'/><inertia ixx='.01' iyy='.01' izz='.01' ixy='0' ixz='0' iyz='0'/></inertial></link></robot>");
            var scene=new ImportedRobotScene(RobotUrdf.parse(file),file,map,new DesktopAssetManager(true),8);
            var wheels=new ArrayList<TireDrive.Wheel>();
            for(float x:new float[]{-.08f,0,.08f})for(int side:new int[]{-1,1})wheels.add(new TireDrive.Wheel("w"+x+side,side<0?"left":"right",new Vector3f(x,-.005f,side*.12f),.0001));
            scene.driveWheels=wheels;
            var spec=new TireFriction.Spec(mu,mu*.8,1,100,.15);
            var model=new TireDrive(world,map,scene,new DifferentialDriveConfig("left","right",.24,.045,-1,1),new TireDriveConfig(spec,spec,List.of(),.0015,.004));world.installTires(model);
            double maxSlip=0;
            for(int i=0;i<Math.round(2/dt);i++){left.integrate(12,dt,Math.round(i*dt*1000));right.integrate(12,dt,Math.round(i*dt*1000));space.update(dt,0);maxSlip=Math.max(maxSlip,model.states().stream().mapToDouble(s->Math.abs(s.slipMps())).max().orElse(0));}
            return new Result(world.getChassisPosition().x,Math.abs(left.getOmegaRadS()),left.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS),maxSlip,(int)model.states().stream().filter(TireDrive.State::supported).count());
        }finally{space.destroy();}
    }
    @Test void lowGripSlipsAirborneWheelsSpinAndWallLoadRaisesCurrent() throws Exception {
        var grip=drive(.9,false,false,1f/120);var slippery=drive(.05,false,false,1f/120);var air=drive(.9,true,false,1f/120);var blocked=drive(3,false,true,1f/120);
        System.out.println("[CONTACT TEST] grip="+grip+" slippery="+slippery+" airborne="+air+" blocked="+blocked);
        assertEquals(6,grip.supported);assertTrue(grip.distance>slippery.distance*2);
        assertTrue(slippery.maxSlip>grip.maxSlip);assertEquals(0,air.supported);assertEquals(0,air.distance,.0001);assertTrue(air.omega>10);
        assertTrue(blocked.distance<.3);assertTrue(blocked.current>grip.current+.2);assertTrue(blocked.omega<grip.omega);
        var fine=drive(.9,false,false,1f/240);assertEquals(grip.distance,fine.distance,.08,"Contact must remain stable when timestep is halved");
    }
    @Test void paddleCollisionsTransportGamePieceInBothMotorDirections() {
        double inward=transport(1), outward=transport(-1);
        System.out.println("[FLEX CONTACT TEST] inward="+inward+" outward="+outward);
        assertTrue(inward>.01,"Positive rotor motion must transport by collision");
        assertTrue(outward<-.01,"Reverse rotor motion must reverse transport");
    }
    private double transport(int direction) {
        PhysicsSpace space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(Vector3f.ZERO);
            var shaft=new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(.002f,.002f,.012f)),1);
            shaft.setKinematic(true);space.add(shaft);
            var flap=new FlexibleFlap(space,shaft,new Transform(),.0045,flexSpec());
            var piece=new PhysicsRigidBody(new com.jme3.bullet.collision.shapes.SphereCollisionShape(.008f),.003f);
            piece.setPhysicsLocation(new Vector3f(.044f,0,0));piece.setEnableSleep(false);piece.setFriction(1);space.add(piece);
            for(int i=0;i<240;i++){shaft.setPhysicsRotation(new Quaternion().fromAngleAxis(direction*i/240f*3,Vector3f.UNIT_Z));space.update(1f/240,0);}
            assertTrue(space.contains(piece));assertTrue(flap.maxDeflectionRad()>.005,"Contact must bend the paddle");
            return piece.getPhysicsLocation().y;
        }finally{space.destroy();}
    }
    @Test void cadRubberDeformsInTheShaftPlaneAndKeepsItsOriginalRestPose() {
        PhysicsSpace space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            var shaft=new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(.002f,.002f,.012f)),0);space.add(shaft);
            Quaternion beamToMesh=new Quaternion().fromAngleAxis(FastMath.HALF_PI,Vector3f.UNIT_Y);
            var flap=new FlexibleFlap(space,shaft,new Transform(Vector3f.ZERO,beamToMesh),.0045,flexSpec());
            var mesh=new com.jme3.scene.shape.Box(.006f,.05f,.003f);
            var geometry=new com.jme3.scene.Geometry("rubber",mesh);
            var original=mesh.getFloatBuffer(com.jme3.scene.VertexBuffer.Type.Position);
            float[] rest=new float[original.limit()];original.duplicate().rewind().get(rest);
            flap.bindVisual(geometry,Transform::new,beamToMesh.inverse());flap.updateVisual();
            var positions=geometry.getMesh().getFloatBuffer(com.jme3.scene.VertexBuffer.Type.Position);
            for(int i=0;i<rest.length;i++)assertEquals(rest[i],positions.get(i),.000001,"Rest mesh must be preserved");
            var tip=flap.segments.get(2).body();
            tip.setPhysicsRotation(new Quaternion().fromAngleAxis(.2f,Vector3f.UNIT_X).mult(beamToMesh));flap.updateVisual();
            double change=0;
            for(int i=0;i<rest.length;i+=3){assertEquals(rest[i],positions.get(i),.000001,"Width/shaft coordinate must not bend");change+=Math.abs(rest[i+2]-positions.get(i+2));}
            assertTrue(change>.001,"Rubber must visibly bend in the radial/tangent plane");
        }finally{space.destroy();}
    }
    @Test void bendingLoadsReactBackOntoTheFreeShaft() {
        PhysicsSpace space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(Vector3f.ZERO);
            var shaft=new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(.01f,.01f,.012f)),.05f);space.add(shaft);
            var flap=new FlexibleFlap(space,shaft,new Transform(),.0045,flexSpec());
            for(int i=0;i<30;i++){flap.segments.get(2).body().applyCentralForce(new Vector3f(.001f,0,0));space.update(1f/240,0);}
            assertTrue(Math.abs(shaft.getAngularVelocity().z)>.01,"Beam constraint must transfer torque to its shaft");
        }finally{space.destroy();}
    }
    static FlexibleIntakeConfig flexSpec(){return new FlexibleIntakeConfig(List.of("flap"),3,.9,.025,.05,.003,.015,.5,.8,1,500,.3,new Vector3f(-1,-1,-1),new Vector3f(1,1,1));}
    @Test void elasticPaddleBendsUnderLoadAndRecovers() {
        PhysicsSpace space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(Vector3f.ZERO);var shaft=new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(.002f,.002f,.012f)),0);space.add(shaft);
            var flap=new FlexibleFlap(space,shaft,new Transform(),.0045,flexSpec());
            var tip=flap.segments.get(2).body();
            for(int i=0;i<240;i++){tip.applyCentralForce(new Vector3f(.025f,0,0));space.update(1f/240,0);}
            double bent=flap.maxDeflectionRad();
            for(int i=0;i<720;i++)space.update(1f/240,0);
            double recovered=flap.maxDeflectionRad();
            System.out.println("[FLEX TEST] bent="+bent+" recovered="+recovered);
            assertTrue(bent>.03);assertTrue(bent<=.85);assertTrue(recovered<bent*.2);assertEquals(.0045,flap.mass(),1e-8);
        }finally{space.destroy();}
    }
}
