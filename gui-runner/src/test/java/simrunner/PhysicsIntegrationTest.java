package simrunner;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.system.NativeLibraryLoader;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import physics.MecanumKinematics;
import physics.MotorSpec;
import simcore.RobotUrdf;
import simcore.SimDcMotorEx;
import simcore.SimServo;
import physics.ServoModel;
import java.util.Map;

import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

class PhysicsIntegrationTest {
    @TempDir Path temp;
    @BeforeAll static void nativePhysics() { NativeLibraryLoader.loadNativeLibrary("bulletjme", true); }
    private static final float DT = 1f / 60;

    @Test void visualOnlyCadIsRejectedForPhysicsWithPreviewGuidance() throws Exception {
        Path file = temp.resolve("preview-only.urdf");
        Files.writeString(file, "<robot name='cad'>" + link("base", "0 0 0", ".1 .1 .1", 1)
            .replace("<collision>", "<visual>").replace("</collision>", "</visual>") + "</robot>");
        PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            ImportedRobotScene scene = new ImportedRobotScene(RobotUrdf.parse(file), file, new HardwareMap(), new DesktopAssetManager(true), 8);
            IllegalArgumentException error = assertThrows(IllegalArgumentException.class, () ->
                new ArticulatedRobot(scene, new PhysicsWorld(null, new Node(), space), new Node(), Vector3f.ZERO));
            assertTrue(error.getMessage().contains("previewRobot"));
        } finally { space.destroy(); }
    }

    @Test void principalFrameProducesCrossAxisAngularResponse() {
        PrincipalInertia inertia = new PrincipalInertia();
        inertia.addRotated(new double[][] {{2, .6, 0}, {.6, 3, 0}, {0, 0, 4}}, new com.jme3.math.Quaternion());
        var principal = inertia.diagonalize("coupled");
        PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(Vector3f.ZERO);
            PhysicsRigidBody body = new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(.1f, .1f, .1f)), 1);
            body.setPhysicsRotation(principal.rotation());
            body.setInverseInertiaLocal(new Vector3f(1 / principal.moments().x,
                1 / principal.moments().y, 1 / principal.moments().z));
            space.add(body);
            body.applyTorqueImpulse(Vector3f.UNIT_X);
            space.update(DT, 0);
            // For [[2,.6],[.6,3]], inverse-inertia response to unit X is [3,-.6]/5.64.
            assertEquals(3 / 5.64, body.getAngularVelocity().x, .02);
            assertEquals(-.6 / 5.64, body.getAngularVelocity().y, .02);
        } finally { space.destroy(); }
    }

    @Test void pushPersistsAndDriveIntoWallStaysFinite() {
        PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(Vector3f.ZERO);
            PhysicsWorld world = new PhysicsWorld(null, new Node(), space);
            world.buildChassis(new Node(), 8.5, Vector3f.ZERO);
            assertEquals(Vector3f.ZERO, world.getChassisPosition());
            world.chassisBody().applyCentralImpulse(new Vector3f(8.5f, 0, 0));
            space.update(DT, 0);
            assertTrue(world.chassisBody().getLinearVelocity().x > 0.5, "Push must not be overwritten");
            for (int i = 0; i < 120; i++) space.update(DT, 0);
            assertTrue(world.getChassisPosition().x > 0.03, "External push must cause displacement");
            PhysicsRigidBody wall = new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(.05f, 1, 1)), 0);
            wall.setPhysicsLocation(new Vector3f(1, 0, 0));
            space.add(wall);
            world.driveChassis(new MecanumKinematics.ChassisVelocity(2, 0, 0), DT);
            for (int i = 0; i < 1800; i++) space.update(i % 2 == 0 ? DT : 1f / 30, 0);
            Vector3f p = world.getChassisPosition();
            assertTrue(Float.isFinite(p.x) && Math.abs(p.x) < 0.85, "Chassis must remain behind the wall: " + p);
        } finally { space.destroy(); }
    }

    @Test void driveDoesNotCancelGravityAndChassisStartsAtRequestedPosition() {
        PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            PhysicsWorld world = new PhysicsWorld(null, new Node(), space);
            world.buildChassis(new Node(), 8.5, new Vector3f(0, 2, 0));
            assertEquals(2, world.getChassisPosition().y, .0001);
            PhysicsRigidBody floor = new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(3, .1f, 3)), 0);
            floor.setPhysicsLocation(new Vector3f(0, -.1f, 0));
            space.add(floor);
            for (int i = 0; i < 20; i++) space.update(DT, 0);
            assertTrue(world.chassisBody().getLinearVelocity().y < -2, "Drive must leave vertical motion to Bullet");
            for (int i = 0; i < 180; i++) space.update(DT, 0);
            assertEquals(.1, world.getChassisPosition().y, .02);
        } finally { space.destroy(); }
    }

    private static String link(String name, String center, String dimensions, double mass) {
        return "<link name='" + name + "'><inertial><origin xyz='" + center + "'/><mass value='" + mass
            + "'/><inertia ixx='.01' iyy='.02' izz='.02' ixy='0' ixz='0' iyz='0'/></inertial>"
            + "<collision><origin xyz='" + center + "'/><geometry><box size='" + dimensions + "'/></geometry></collision></link>";
    }

    @Test void physicalSlideStallsAgainstObstacleAndEncoderResetDoesNotMoveIt() throws Exception {
        Path urdfPath = temp.resolve("slide.urdf");
        Files.writeString(urdfPath, "<robot name='slide'>" + link("base", "0 0 0", ".1 .1 .1", 5)
            + link("carriage", ".1 0 0", ".1 .1 .1", 1)
            + "<joint name='slide' type='prismatic'><parent link='base'/><child link='carriage'/><origin xyz='.2 0 0'/><axis xyz='1 0 0'/><limit lower='0' upper='.5'/></joint>"
            + "<transmission name='tx'><joint name='slide'/><actuator name='motor'><mechanicalReduction>10</mechanicalReduction></actuator></transmission></robot>");
        PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(Vector3f.ZERO);
            HardwareMap map = new HardwareMap();
            SimDcMotorEx motor = new SimDcMotorEx("motor", new MotorSpec("test", 1, 2, 9, 30, 12, 500));
            map.register("motor", motor);
            ImportedRobotScene scene = new ImportedRobotScene(RobotUrdf.parse(urdfPath), urdfPath, map, new DesktopAssetManager(true), 8);
            PhysicsWorld world = new PhysicsWorld(null, new Node(), space);
            ArticulatedRobot robot = new ArticulatedRobot(scene, world, new Node(), Vector3f.ZERO);
            assertEquals(Vector3f.ZERO, world.getChassisPosition());
            world.chassisBody().setMass(0); // anchor the test fixture
            PhysicsRigidBody obstacle = new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(.05f, 1, 1)), 0);
            obstacle.setPhysicsLocation(new Vector3f(.65f, 0, 0));
            space.add(obstacle);
            motor.setPower(.5);
            for (int i = 0; i < 600; i++) {
                space.update(DT, 0);
                motor.integrate(12, DT, Math.round(i * DT * 1000));
            }
            double stopped = robot.jointPosition("slide");
            assertTrue(stopped > .05 && stopped < .35, "Slide must stop before obstacle, q=" + stopped);
            int ticks = motor.getCurrentPosition();
            assertTrue(ticks > 50, "Encoder must measure actual movement");
            assertTrue(motor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS) > 3,
                "A stalled powered motor must draw current");
            for (int i = 600; i < 720; i++) {
                space.update(DT, 0);
                motor.integrate(12, DT, Math.round(i * DT * 1000));
            }
            assertEquals(stopped, robot.jointPosition("slide"), .01, "Contact must stall instead of passing through");
            assertEquals(ticks, motor.getCurrentPosition(), 10, "Stalled encoder must stop accumulating");
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
            space.update(DT, 0);
            motor.integrate(12, DT, 7000);
            assertEquals(stopped, robot.jointPosition("slide"), .01);
            assertEquals(0, motor.getCurrentPosition(), 5);
            assertEquals(1, space.getRigidBodyList().stream().filter(PhysicsRigidBody::isDynamic).mapToDouble(PhysicsRigidBody::getMass).sum(), .01);
        } finally { space.destroy(); }
    }

    @Test void fieldStartYawRotatesArticulatedBodiesAndJointAxisTogether() throws Exception {
        Path path=temp.resolve("placed-slide.urdf");
        Files.writeString(path,"<robot name='placed'>"+link("base","0 0 0",".1 .1 .1",5)+link("slide","0 0 0",".05 .05 .05",1)
            +"<joint name='axis' type='prismatic'><parent link='base'/><child link='slide'/><origin xyz='.3 0 0'/><axis xyz='1 0 0'/><limit lower='0' upper='.5'/></joint>"
            +"<transmission name='tx'><joint name='axis'/><actuator name='motor'/></transmission></robot>");
        PhysicsSpace space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(Vector3f.ZERO);HardwareMap map=new HardwareMap();SimDcMotorEx motor=new SimDcMotorEx("motor",new MotorSpec("test",1,2,9,30,12,500));map.register("motor",motor);
            ImportedRobotScene scene=new ImportedRobotScene(RobotUrdf.parse(path),path,map,new DesktopAssetManager(true),8);
            PhysicsWorld world=new PhysicsWorld(null,new Node(),space);
            var robot=new ArticulatedRobot(scene,world,new Node(),new Vector3f(1,2,3),java.util.Map.of(),new com.jme3.math.Quaternion().fromAngleAxis((float)Math.PI/2,Vector3f.UNIT_Y));
            assertEquals(1,world.getChassisPosition().x,.001);assertEquals(2,world.getChassisPosition().y,.001);assertEquals(3,world.getChassisPosition().z,.001);
            assertEquals(-1,world.getChassisRotation().mult(Vector3f.UNIT_X).z,.001);
            assertEquals(2.7,robot.bodyForLink("slide").getPhysicsLocation().z,.001);
            world.chassisBody().setMass(0);motor.setPower(.5);
            for(int i=0;i<300;i++)space.update(DT,0);
            assertTrue(robot.bodyForLink("slide").getPhysicsLocation().z<2.65,"Placed slider must move on rotated world axis");
            assertEquals(1,robot.bodyForLink("slide").getPhysicsLocation().x,.005);
        }finally{space.destroy();}
    }

    @Test void nonAdjacentTipCollidesWithOwnChassis() throws Exception {
        Path file = temp.resolve("self-contact.urdf");
        Files.writeString(file, "<robot name='self-contact'>" + link("base", "0 0 0", ".4 .2 .2", 5)
            + link("arm", "0 0 0", ".05 .05 .05", 1)
            + link("tip", "0 0 0", ".1 .1 .1", .5)
            + "<joint name='anchor' type='prismatic'><parent link='base'/><child link='arm'/><origin xyz='.45 0 0'/><axis xyz='1 0 0'/><limit lower='0' upper='0'/></joint>"
            + "<joint name='retract' type='prismatic'><parent link='arm'/><child link='tip'/><origin xyz='.15 0 0'/><axis xyz='-1 0 0'/><limit lower='0' upper='.5'/></joint>"
            + "<transmission name='tip-tx'><joint name='retract'/><actuator name='tip_motor'/></transmission></robot>");
        PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(Vector3f.ZERO);
            HardwareMap map = new HardwareMap();
            SimDcMotorEx motor = new SimDcMotorEx("tip_motor", new MotorSpec("test", 1, 3, 9, 30, 12, 500));
            map.register("tip_motor", motor);
            ImportedRobotScene scene = new ImportedRobotScene(RobotUrdf.parse(file), file, map, new DesktopAssetManager(true), 8);
            PhysicsWorld world = new PhysicsWorld(null, new Node(), space);
            ArticulatedRobot robot = new ArticulatedRobot(scene, world, new Node(), Vector3f.ZERO);
            world.chassisBody().setMass(0);
            assertEquals(1, world.chassisBody().countIgnored()); // adjacent arm only
            motor.setPower(.6);
            for (int i = 0; i < 600; i++) space.update(DT, 0);
            double position = robot.jointPosition("retract");
            assertTrue(position > .15 && position < .42, "Nonadjacent tip must stop against the base: " + position);
            assertEquals(3, space.countJoints());
        } finally { space.destroy(); }
    }

    @Test void specifiedServoStallsUnderContactAndReversesDirection() throws Exception {
        Path file = temp.resolve("servo.urdf");
        Files.writeString(file, "<robot name='servo'>" + link("base", "0 0 0", ".1 .1 .1", 5)
            + link("carriage", ".1 0 0", ".1 .1 .1", 1)
            + "<joint name='slide' type='prismatic'><parent link='base'/><child link='carriage'/><origin xyz='.2 0 0'/><axis xyz='1 0 0'/><limit lower='0' upper='.5' effort='3'/></joint>"
            + "<transmission name='tx'><joint name='slide'/><actuator name='servo'><mechanicalReduction>10</mechanicalReduction></actuator></transmission></robot>");
        PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(Vector3f.ZERO);
            HardwareMap map = new HardwareMap();
            SimServo servo = new SimServo("servo"); map.register("servo", servo);
            ImportedRobotScene scene = new ImportedRobotScene(RobotUrdf.parse(file), file, map, new DesktopAssetManager(true), 8);
            PhysicsWorld world = new PhysicsWorld(null, new Node(), space);
            assertThrows(IllegalArgumentException.class,
                () -> new ArticulatedRobot(scene, world, new Node(), Vector3f.ZERO));
            ServoModel.Spec spec = new ServoModel.Spec(.4, 4, Math.PI, 8, .5, .01);
            ArticulatedRobot robot = new ArticulatedRobot(scene, world, new Node(), Vector3f.ZERO, Map.of("servo", spec));
            world.chassisBody().setMass(0);
            PhysicsRigidBody obstacle = new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(.05f, 1, 1)), 0);
            obstacle.setPhysicsLocation(new Vector3f(.65f, 0, 0));
            space.add(obstacle);
            servo.setPosition(1);
            for (int i = 0; i < 600; i++) space.update(DT, 0);
            assertTrue(robot.jointPosition("slide") > .05 && robot.jointPosition("slide") < .35);
            servo.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
            for (int i = 0; i < 600; i++) space.update(DT, 0);
            assertEquals(0, robot.jointPosition("slide"), .02);
        } finally { space.destroy(); }
    }

    @Test void hingeHonorsLimitAndNestedSlideRemainsConnected() throws Exception {
        Path file = temp.resolve("nested.urdf");
        Files.writeString(file, "<robot name='nested'>" + link("base", "0 0 0", ".1 .1 .1", 5)
            + link("arm", ".15 0 0", ".3 .05 .05", 1)
            + link("tip", ".05 0 0", ".1 .05 .05", .5)
            + "<joint name='hinge' type='revolute'><parent link='base'/><child link='arm'/><origin rpy='.2 .3 .4'/><axis xyz='0 0 1'/><limit lower='0' upper='.7'/></joint>"
            + "<joint name='slide' type='prismatic'><parent link='arm'/><child link='tip'/><origin xyz='.3 0 0'/><axis xyz='1 0 0'/><limit lower='0' upper='.2'/></joint>"
            + "<transmission name='tx1'><joint name='hinge'/><actuator name='armMotor'/></transmission>"
            + "<transmission name='tx2'><joint name='slide'/><actuator name='slideMotor'><mechanicalReduction>10</mechanicalReduction></actuator></transmission></robot>");
        PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(Vector3f.ZERO);
            HardwareMap map = new HardwareMap();
            MotorSpec spec = new MotorSpec("test", 1, 2, 9, 30, 12, 500);
            SimDcMotorEx arm = new SimDcMotorEx("armMotor", spec), slide = new SimDcMotorEx("slideMotor", spec);
            map.register("armMotor", arm); map.register("slideMotor", slide);
            ImportedRobotScene scene = new ImportedRobotScene(RobotUrdf.parse(file), file, map, new DesktopAssetManager(true), 8);
            PhysicsWorld world = new PhysicsWorld(null, new Node(), space);
            ArticulatedRobot robot = new ArticulatedRobot(scene, world, new Node(), Vector3f.ZERO);
            assertEquals(6.5, space.getRigidBodyList().stream().mapToDouble(PhysicsRigidBody::getMass).sum(), .001);
            world.chassisBody().setMass(0);
            arm.setPower(.5); slide.setPower(.5);
            for (int i = 0; i < 1200; i++) space.update(DT, 0);
            assertEquals(.7, robot.jointPosition("hinge"), .04);
            assertEquals(.2, robot.jointPosition("slide"), .02);
            assertEquals(3, space.countJoints());
            arm.setPower(-.5); slide.setPower(-.5);
            for (int i = 0; i < 1200; i++) space.update(DT, 0);
            assertEquals(0, robot.jointPosition("hinge"), .04);
            assertEquals(0, robot.jointPosition("slide"), .02);
            for (PhysicsRigidBody body : space.getRigidBodyList()) {
                Vector3f p = body.getPhysicsLocation();
                assertTrue(Float.isFinite(p.x) && p.length() < 2, "Nested mechanism must remain connected: " + p);
            }
        } finally { space.destroy(); }
    }
}
