package simrunner;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.system.NativeLibraryLoader;
import com.qualcomm.robotcore.hardware.DcMotor;
import simcore.*;
import java.nio.file.Path;

/** Reproducible fixed-timestep differential CAD check without an OpenGL window. */
public final class RobotPhysicsProbe {
    public static void main(String[] args) throws Exception {
        if (args.length != 1) throw new IllegalArgumentException("Usage: RobotPhysicsProbe <preparedProject>");
        Path project = Path.of(args[0]);
        SimConfig config = SimConfig.load(project);
        if (config.drive == null || config.intake == null) throw new IllegalArgumentException("Probe requires differential drive and motor intake");
        NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
        PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            var assets = new DesktopAssetManager(true);
            Node field = new Node();
            PhysicsWorld world = new PhysicsWorld(assets, field, space);
            world.buildFieldBoundary();
            var map = HardwareMapBuilder.build(RobotConfigXml.parse(project.resolve(config.robotConfig).toFile()),
                PresetRobotConfig.load(project.resolve(config.presetMotors)));
            RobotUrdf urdf = RobotUrdf.parse(project.resolve(config.urdf));
            if (config.totalMassKg != null) urdf = urdf.withTotalMassKg(config.totalMassKg);
            urdf.validateHardwareMap(map);
            ImportedRobotScene scene = new ImportedRobotScene(urdf, project.resolve(config.urdf), map, assets, config.vhacdMaxHulls,
                java.util.Set.copyOf(config.drive.motorNames()));
            ArticulatedRobot robot = new ArticulatedRobot(scene, world, field, new Vector3f(0, (float) config.startHeightM, 0));
            SimDcMotorEx left = map.get(SimDcMotorEx.class, config.drive.leftMotor()), right = map.get(SimDcMotorEx.class, config.drive.rightMotor());
            left.setDirection(DcMotor.Direction.REVERSE);
            var motors = map.getAll(SimDcMotorEx.class);
            SimDcMotorEx intake = map.get(SimDcMotorEx.class, config.intake.motor());
            System.out.println("[PROBE] initial rotation="+world.getChassisRotation()+" origin="+world.robotPointWorld(Vector3f.ZERO));
            left.setPower(.28); right.setPower(.28); intake.setPower(.7);
            for (int i=0;i<180;i++) {
                for (var m:motors) m.integrate(12, 1.0/60, Math.round(i*1000.0/60));
                world.driveChassis(config.drive.velocity(map), 1f/60);
                space.update(1f/60,0);
                if (i%60==0)System.out.println("[PROBE] i="+i+" target="+config.drive.velocity(map)+" omega="+world.getChassisAngularVelocity()+" forward="+world.getChassisRotation().mult(Vector3f.UNIT_X));
            }
            Vector3f forward=world.getChassisRotation().mult(Vector3f.UNIT_X);
            double yaw=Math.atan2(-forward.z,forward.x);
            System.out.println("[PROBE] straight="+world.getChassisPosition()+" yaw="+yaw+" joints="+robot.jointPositions());
            require(world.getChassisPosition().x>.5 && Math.abs(yaw)<.1 && Math.abs(world.getChassisPosition().z)<.05,"Straight drive deviated");
            require(Math.abs(robot.jointPosition("intake_upper_joint")-robot.jointPosition("intake_lower_joint"))<.1,"Chain follower lost phase");
            require(robot.jointPosition("intake_upper_joint")>5 && intake.getCurrentPosition()>100,"Intake/encoder did not advance");
            Vector3f point=world.robotPointWorld(config.intake.point());
            world.buildGamePiece(point);
            world.updateIntake(config.intake.active(map),point,config.intake.captureRadiusM());
            require(world.isPieceHeld(),"Powered intake did not capture");
            Vector3f heldPoint=point.add(.1f,0,0);
            world.updateIntake(true,heldPoint,config.intake.captureRadiusM());
            world.updateIntake(false,heldPoint,config.intake.captureRadiusM());
            require(!world.isPieceHeld() && world.getGamePiecePosition().distance(heldPoint)<.001,"Release position wrong");
            System.out.println("[PROBE] capture/release passed");
            left.setPower(-.18);right.setPower(.18);
            for (int i=180;i<240;i++) {
                for(var m:motors)m.integrate(12,1.0/60,Math.round(i*1000.0/60));
                world.driveChassis(config.drive.velocity(map),1f/60);space.update(1f/60,0);
            }
            forward=world.getChassisRotation().mult(Vector3f.UNIT_X);yaw=Math.atan2(-forward.z,forward.x);
            require(yaw>.4,"Differential turn failed");
            System.out.println("[PROBE] turn yaw="+yaw+" PASS");
        } finally {space.destroy();}
    }
    private static void require(boolean value,String message) {if(!value)throw new IllegalStateException(message);}
}
