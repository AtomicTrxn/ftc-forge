package simrunner;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.*;
import java.util.*;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.math.*;
import simcore.*;
import static org.junit.jupiter.api.Assertions.*;
class RotatingWheelPhysicsTest {
    @TempDir Path tmp;
    @BeforeAll static void natives(){com.jme3.system.NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    private GuidedSetupController model(boolean suspension)throws Exception {return SyntheticRobots.rotatingRobot(tmp,suspension);}
    private void ticks(RobotMotionDemo d,double seconds){for(int i=0;i<seconds*480;i++){for(var m:d.setup.hardware().getAll(SimDcMotorEx.class))m.integrate(12,1./480,Math.round(i*1000./480));d.space.update(1f/480,0);}}
    @Test void wheelsActuallyRotateSharedShaftsDriveAndAirborneHasNoArtificialAcceleration()throws Exception {
        var c=model(false);try(var d=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true))) {
            ticks(d,1);assertFalse(d.world.driveControllerEnabled);assertEquals(6,d.space.countRigidBodies());
            var diagnostics=new PhysicsDiagnostics(d.world,d.scene,d.robot,d.setup.hardware(),d.setup.plan().driveNames).capture();assertEquals(4,diagnostics.wheels().size());assertTrue(diagnostics.wheels().stream().allMatch(PhysicsDiagnostics.Wheel::supported));assertTrue(diagnostics.notes().stream().noneMatch(n->n.contains("legacy")));
            assertEquals(3,d.space.getRigidBodyList().stream().filter(b->b.isDynamic()).mapToDouble(PhysicsRigidBody::getMass).sum(),1e-5);
            d.setup.hardware().get(SimDcMotorEx.class,"leftDrive").setPower(-.3);d.setup.hardware().get(SimDcMotorEx.class,"rightDrive").setPower(.3);var start=d.world.getChassisPosition();ticks(d,1);
            double travel=d.world.getChassisPosition().x-start.x;System.out.println("[ROTATING] travel="+travel);assertTrue(travel>.05&&travel<1);
            assertTrue(Math.abs(d.robot.jointPosition("wheel_joint0"))>1);assertEquals(d.robot.jointPosition("wheel_joint0"),d.robot.jointPosition("wheel_joint2"),.02);
            assertTrue(Math.abs(d.setup.hardware().get(SimDcMotorEx.class,"leftDrive").getPhysicalShaftRadians())>1);
            d.space.setGravity(Vector3f.ZERO);for(var b:d.space.getRigidBodyList())if(b.isDynamic()){b.setPhysicsLocation(b.getPhysicsLocation().add(0,3,0));b.setLinearVelocity(Vector3f.ZERO);b.setAngularVelocity(Vector3f.ZERO);}
            ticks(d,.5);assertTrue(d.world.chassisBody().getLinearVelocity().length()<.02,"Native shaft torque must not invent airborne traction");
        }
    }
    @Test void rampAndTipOverChangeActualOrientationAndImu()throws Exception {
        var c=model(false);try(var d=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true))) {
            d.space.remove(SimulatorValidation.floor(d));float slope=.15f;
            var ramp=new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(3,.02f,3)),0);ramp.setPhysicsRotation(new Quaternion().fromAngleAxis(slope,Vector3f.UNIT_Z));ramp.setPhysicsLocation(new Vector3f(0,-.03f,0));ramp.setFriction(.6f);d.space.add(ramp);
            for(var b:d.space.getRigidBodyList())if(b.isDynamic())b.setPhysicsLocation(b.getPhysicsLocation().add(0,.08f,0));ticks(d,3);
            var imu=new SimIMU("imu");imu.setLatencyMs(0);RobotOrientation.update(imu,d.world.getChassisRotation(),d.world.getChassisAngularVelocity(),10);
            double pitch=imu.getRobotYawPitchRollAngles().getPitch(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);System.out.println("[ROTATING] ramp pitch="+pitch);assertEquals(-slope,pitch,.06);
            d.world.chassisBody().applyTorqueImpulse(new Vector3f(1,0,0));ticks(d,.7);
            var up=d.world.getChassisRotation().mult(Vector3f.UNIT_Y);assertTrue(up.y<.7,"Free chassis should physically tip under sufficient torque");
        }
    }
    @Test void suspensionCompressesWithNativeSpringAndRespectsTravel()throws Exception {
        var c=model(true);try(var d=new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true))) {
            ticks(d,3);for(int i=0;i<4;i++){double q=d.robot.jointPosition("spring"+i);System.out.println("[SPRING] "+i+"="+q);assertTrue(q>.002&&q<.04);}
            assertTrue(d.robot.diagnosticJoints().stream().filter(j->j.name().startsWith("spring")).allMatch(ArticulatedRobot.DiagnosticJoint::effortIsEstimate));
            for(var b:d.space.getRigidBodyList())if(b.isDynamic()){assertTrue(Vector3f.isValidVector(b.getPhysicsLocation()));assertTrue(b.getLinearVelocity().length()<.1);}
        }
    }
}
