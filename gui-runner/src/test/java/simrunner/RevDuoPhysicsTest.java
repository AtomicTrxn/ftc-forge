package simrunner;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.system.NativeLibraryLoader;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import physics.MotorSpec;
import simcore.RobotUrdf;
import simcore.SimDcMotorEx;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Set;
import static org.junit.jupiter.api.Assertions.*;

class RevDuoPhysicsTest {
    @TempDir Path temp;
    @BeforeAll static void nativePhysics(){NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    private static final float DT=1f/120;
    @Test void mountingSignsDoNotFollowOpModeDirectionAndWheelAnglesSurviveReset() {
        HardwareMap map=new HardwareMap();
        var left=motor("customLeft"); var right=motor("customRight");
        map.register("customLeft",left);map.register("customRight",right);
        var drive=new DifferentialDriveConfig("customLeft","customRight",.4,.045,-1,1);
        left.setDirection(DcMotor.Direction.REVERSE);left.setPower(.4);right.setPower(.4);
        for(int i=0;i<300;i++){left.integrate(12,DT,i*8);right.integrate(12,DT,i*8);}
        assertTrue(left.getCurrentPosition()>100); assertTrue(right.getCurrentPosition()>100);
        assertTrue(left.getVelocity()>0);
        assertTrue(drive.velocity(map).vx>.4);assertEquals(0,drive.velocity(map).omega,1e-9);
        double physicalAngle=left.getPhysicalShaftRadians();
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        assertEquals(physicalAngle,left.getPhysicalShaftRadians());
        left.setDirection(DcMotor.Direction.FORWARD);left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);left.setPower(.4);
        for(int i=0;i<300;i++)left.integrate(12,DT,3000+i*8);
        assertTrue(drive.velocity(map).omega>1,"Wrong mounting direction should physically turn instead of being silently undone");
    }
    @Test void driveAndIntakeUseRobotFrameInsteadOfPrincipalAxes() {
        PhysicsSpace space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try{
            space.setGravity(Vector3f.ZERO);
            var world=new PhysicsWorld(null,new Node(),space);
            world.buildChassis(new Node(),3,Vector3f.ZERO);
            Quaternion principal=new Quaternion().fromAngleAxis(.7f,Vector3f.UNIT_Y);
            world.chassisBody().setPhysicsRotation(principal);
            world.setRobotFrame(principal.inverse(),principal.inverse().mult(new Vector3f(-.1f,0,0)));
            assertTrue(new Vector3f(.2f,0,0).distance(world.robotPointWorld(new Vector3f(.3f,0,0)))<.00001);
            world.driveChassis(new physics.MecanumKinematics.ChassisVelocity(1,0,0),DT);
            for(int i=0;i<120;i++)space.update(DT,0);
            assertTrue(world.getChassisPosition().x>.7);
            assertEquals(0,world.getChassisPosition().z,.001);
        }finally{space.destroy();}
    }
    @Test void customWheelBindingsStayBallastAndOneMotorDrivesBothIntakeShafts() throws Exception {
        Path file=temp.resolve("robot.urdf");
        Files.writeString(file,"<robot name='coupled'>"+link("base",".3 .2 .2",3)+link("wheel",".05 .05 .05",.1)
            +link("upper",".05 .2 .05",.1)+link("lower",".05 .2 .05",.1)
            +joint("wheelJoint","wheel","0 .3 0","")
            +joint("upperJoint","upper",".3 0 .2","")
            +joint("lowerJoint","lower",".3 0 .1","<mimic joint='upperJoint' multiplier='1'/>")
            +tx("wheelJoint","customLeft")+tx("upperJoint","intake")+"</robot>");
        HardwareMap map=new HardwareMap();var intake=motor("intake");map.register("intake",intake);map.register("customLeft",motor("customLeft"));
        PhysicsSpace space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try{
            space.setGravity(Vector3f.ZERO);
            var scene=new ImportedRobotScene(RobotUrdf.parse(file),file,map,new DesktopAssetManager(true),8,Set.of("customLeft"));
            var world=new PhysicsWorld(null,new Node(),space);
            var robot=new ArticulatedRobot(scene,world,new Node(),Vector3f.ZERO);
            assertEquals(3,space.countRigidBodies()); assertEquals(4,space.countJoints());
            assertTrue(scene.wheelLinks.contains("wheel"));
            assertEquals(3.3,space.getRigidBodyList().stream().mapToDouble(b->b.getMass()).sum(),.001);
            world.chassisBody().setMass(0);
            var capture=new MotorIntakeConfig("intake",1,1,new Vector3f(.3f,.1f,0),.12f);
            assertFalse(capture.active(map));intake.setPower(.5);
            for(int i=0;i<600;i++){space.update(DT,0);intake.integrate(12,DT,i*8);}
            assertTrue(robot.jointPosition("upperJoint")>10); assertEquals(robot.jointPosition("upperJoint"),robot.jointPosition("lowerJoint"),.08);
            assertTrue(intake.getCurrentPosition()>100);assertTrue(capture.active(map));
            intake.setPower(-.5);
            for(int i=600;i<900;i++){space.update(DT,0);intake.integrate(12,DT,i*8);}
            assertFalse(capture.active(map));
        }finally{space.destroy();}
    }
    @Test void elasticMotorWithReflectedInertiaLoadsReversesAndDistinguishesBrakeFromFloat() throws Exception {
        Path file=temp.resolve("elastic.urdf");
        Files.writeString(file,"<robot name='elastic'>"+link("base",".3 .2 .2",3)+link("upper",".05 .2 .05",.1)
            +link("lower",".05 .2 .05",.1)+joint("upperJoint","upper",".3 0 .2","")
            +joint("lowerJoint","lower",".3 0 .1","<mimic joint='upperJoint' multiplier='1'/>")+tx("upperJoint","intake")+"</robot>");
        HardwareMap map=new HardwareMap();var intake=motor("intake");map.register("intake",intake);intake.configureFriction(0,0);
        PhysicsSpace space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(Vector3f.ZERO);
            var scene=new ImportedRobotScene(RobotUrdf.parse(file),file,map,new DesktopAssetManager(true),8);
            scene.flexibleIntake=ContactModelsTest.flexSpec();
            var world=new PhysicsWorld(null,new Node(),space);
            var robot=new ArticulatedRobot(scene,world,new Node(),Vector3f.ZERO);
            robot.addReflectedShaftInertia("intake",.0015);world.chassisBody().setMass(0);
            intake.setPower(.5);elasticTicks(space,intake,4);
            double free=intake.getOmegaRadS(),freeCurrent=intake.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            robot.setShaftLoad("intake",.3);elasticTicks(space,intake,4);
            double loaded=intake.getOmegaRadS(),loadedCurrent=intake.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            assertEquals(15,free,1);assertTrue(loaded<free-3);assertTrue(loadedCurrent>freeCurrent+.5);
            robot.setShaftLoad("intake",0);intake.setPower(-.5);elasticTicks(space,intake,4);
            assertTrue(intake.getOmegaRadS()<-10);assertEquals(robot.jointPosition("upperJoint"),robot.jointPosition("lowerJoint"),.001);
            intake.setPower(0);intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            double spinning=intake.getOmegaRadS();elasticTicks(space,intake,1);
            assertEquals(spinning,intake.getOmegaRadS(),.1,"FLOAT must preserve unloaded coasting");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);elasticTicks(space,intake,6);
            assertTrue(Math.abs(intake.getOmegaRadS())<.1,"BRAKE must electrically damp the shaft");
        }finally{space.destroy();}
    }
    private static void elasticTicks(PhysicsSpace space,SimDcMotorEx motor,double seconds) {
        float dt=1f/480;
        for(int i=0;i<Math.round(seconds/dt);i++){motor.integrate(12,dt,Math.round(i*dt*1000));space.update(dt,0);}
    }
    @Test void captureReleaseMovesThePhysicsBodyToTheHeldPosition() {
        PhysicsSpace space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            var world=new PhysicsWorld(new DesktopAssetManager(true),new Node(),space);
            world.buildChassis(new Node(),3,Vector3f.ZERO);
            Vector3f start=new Vector3f(.3f,.2f,0), end=new Vector3f(.6f,.2f,.1f);
            world.buildGamePiece(start);world.updateIntake(true,start,.12f);assertTrue(world.isPieceHeld());
            world.updateIntake(true,end,.12f);world.updateIntake(false,end,.12f);
            assertFalse(world.isPieceHeld());assertEquals(end,world.getGamePiecePosition());
            space.update(DT,0); assertTrue(world.getGamePiecePosition().distance(end)<.01);
        }finally{space.destroy();}
    }
    @Test void rejectsInvalidMimicBindings() throws Exception {
        String xml="<robot name='mimic'>"+link("base",".3 .2 .2",3)+link("upper",".05 .2 .05",.1)
            +link("lower",".05 .2 .05",.1)+joint("upperJoint","upper",".3 0 .2","")
            +joint("lowerJoint","lower",".3 0 .1","<mimic joint='upperJoint' multiplier='1'/>")+tx("upperJoint","intake")+"</robot>";
        Path file=temp.resolve("mimic.urdf"); Files.writeString(file,xml);
        assertEquals("upperJoint",RobotUrdf.parse(file).joints.get("lowerJoint").mimic());
        for(String invalid:new String[]{xml.replace("joint='upperJoint' multiplier", "joint='missing' multiplier"),
                xml.replace("multiplier='1'", "multiplier='2'"),xml.replace("multiplier='1'", "multiplier='1' offset='.1'"),
                xml.replace("</robot>",tx("lowerJoint","secondMotor")+"</robot>")}) {
            Files.writeString(file,invalid);assertThrows(IllegalArgumentException.class,()->RobotUrdf.parse(file));
        }
    }
    private static SimDcMotorEx motor(String name){return new SimDcMotorEx(name,new MotorSpec("test",1,2,8,30,12,500));}
    private static String link(String name,String size,double mass){return "<link name='"+name+"'><inertial><mass value='"+mass+"'/><inertia ixx='.01' iyy='.02' izz='.03' ixy='0' ixz='0' iyz='0'/></inertial><collision><geometry><box size='"+size+"'/></geometry></collision></link>";}
    private static String joint(String name,String child,String position,String extra){return "<joint name='"+name+"' type='continuous'><parent link='base'/><child link='"+child+"'/><origin xyz='"+position+"'/><axis xyz='0 1 0'/>"+extra+"</joint>";}
    private static String tx(String joint,String actuator){return "<transmission name='tx_"+joint+"'><joint name='"+joint+"'/><actuator name='"+actuator+"'/></transmission>";}
}
