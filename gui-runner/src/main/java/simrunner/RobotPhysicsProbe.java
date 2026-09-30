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
            scene.tireContacts = config.tires != null;
            scene.flexibleIntake = config.flexibleIntake;
            ArticulatedRobot robot = new ArticulatedRobot(scene, world, field, new Vector3f(0, (float) config.startHeightM, 0));
            if (config.tires != null) world.installTires(new TireDrive(world,map,scene,config.drive,config.tires));
            if (config.flexibleIntake != null) world.installFlexibleIntake(new FlexibleIntake(world,scene,robot,config.flexibleIntake));
            SimDcMotorEx left = map.get(SimDcMotorEx.class, config.drive.leftMotor()), right = map.get(SimDcMotorEx.class, config.drive.rightMotor());
            left.setDirection(DcMotor.Direction.REVERSE);
            var motors = map.getAll(SimDcMotorEx.class);
            SimDcMotorEx intake = map.get(SimDcMotorEx.class, config.intake.motor());
            System.out.println("[PROBE] initial rotation="+world.getChassisRotation()+" origin="+world.robotPointWorld(Vector3f.ZERO));
            if (config.flexibleIntake != null) {
                world.buildGamePiece(world.robotPointWorld(new Vector3f(.42f,.04f,0)));
                world.gamePieceBody().setPhysicsRotation(new com.jme3.math.Quaternion().fromAngleAxis(com.jme3.math.FastMath.HALF_PI,Vector3f.UNIT_X));
            }
            left.setPower(.28); right.setPower(.28); intake.setPower(.7);
            float dt=config.flexibleIntake == null ? 1f/60 : 1f/480;
            int ticks=Math.round(3/dt);
            boolean everContained=false;
            double peakDeflection=0;
            for (int i=0;i<ticks;i++) {
                for (var m:motors) m.integrate(12, dt, Math.round(i*dt*1000));
                world.driveChassis(config.drive.velocity(map), dt);
                space.update(dt,0);
                if(config.flexibleIntake!=null)world.updateIntake(true,world.robotPointWorld(config.intake.point()),config.intake.captureRadiusM());
                if(config.flexibleIntake!=null){everContained|=world.isPieceHeld();peakDeflection=Math.max(peakDeflection,world.flexibleIntake().maxDeflectionRad());}
                if (i%120==0)System.out.println("[PROBE] i="+i+" target="+config.drive.velocity(map)+" omega="+world.getChassisAngularVelocity()+" forward="+world.getChassisRotation().mult(Vector3f.UNIT_X));
            }
            Vector3f forward=world.getChassisRotation().mult(Vector3f.UNIT_X);
            double yaw=Math.atan2(-forward.z,forward.x);
            System.out.println("[PROBE] straight="+world.getChassisPosition()+" yaw="+yaw+" joints="+robot.jointPositions());
            require(world.getChassisPosition().x>.5 && Math.abs(yaw)<.1 && Math.abs(world.getChassisPosition().z)<.05,"Straight drive deviated");
            require(Math.abs(robot.jointPosition("intake_upper_joint")-robot.jointPosition("intake_lower_joint"))<.1,"Chain follower lost phase");
            require(robot.jointPosition("intake_upper_joint")>(config.flexibleIntake==null?5:.5) && intake.getCurrentPosition()>20,"Intake/encoder did not advance");
            if (config.flexibleIntake != null) {
                System.out.println("[PROBE] tire states="+(world.tireDrive()==null?java.util.List.of():world.tireDrive().states())+" flex="+world.flexibleIntake().maxDeflectionRad());
                require(Math.abs(space.getRigidBodyList().stream().mapToDouble(b->b.getMass()).sum()-urdf.totalMassKg()-.1)<.0001,"Flexible mass counted twice");
                System.out.println("[PROBE CONTACT] everContained="+everContained+" peakBend="+peakDeflection+" current="+intake.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS)+" piece="+world.getGamePiecePosition()+" robot="+world.robotPointWorld(Vector3f.ZERO)+" contained="+world.isPieceHeld());
                left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                left.setPower(0);right.setPower(0);intake.setPower(-.7);
                for(int i=ticks;i<ticks+Math.round(2/dt);i++){for(var m:motors)m.integrate(12,dt,Math.round(i*dt*1000));space.update(dt,0);world.updateIntake(false,world.robotPointWorld(config.intake.point()),config.intake.captureRadiusM());}
                System.out.println("[PROBE CONTACT] reverse piece="+world.getGamePiecePosition()+" robot="+world.robotPointWorld(Vector3f.ZERO)+" contained="+world.isPieceHeld());
                require(space.contains(world.gamePieceBody()),"Contact intake removed the physical game piece");
                Vector3f piece=world.getGamePiecePosition();
                require(Float.isFinite(piece.x)&&Float.isFinite(piece.y)&&Float.isFinite(piece.z)&&piece.y>-.01&&Math.abs(piece.x)<1.8&&Math.abs(piece.z)<1.8,
                    "Contact game piece escaped the field or became nonfinite: "+piece);
                left.setPower(-.18);right.setPower(.18);intake.setPower(0);
                for(int i=0;i<Math.round(1/dt);i++){for(var m:motors)m.integrate(12,dt,Math.round((ticks+i)*dt*1000));space.update(dt,0);}
                forward=world.getChassisRotation().mult(Vector3f.UNIT_X);double turned=Math.atan2(-forward.z,forward.x);
                require(turned>yaw+.3,"Contact drive failed its skid-steering turn");
                System.out.println("[PROBE CONTACT] turnYaw="+turned+" PASS (physical contact; containment="+everContained+")");
                return;
            }
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
                world.driveChassis(config.drive.velocity(map),1f/60);space.update(dt,0);
            }
            forward=world.getChassisRotation().mult(Vector3f.UNIT_X);yaw=Math.atan2(-forward.z,forward.x);
            require(yaw>.4,"Differential turn failed");
            System.out.println("[PROBE] turn yaw="+yaw+" PASS");
        } finally {space.destroy();}
    }
    private static void require(boolean value,String message) {if(!value)throw new IllegalStateException(message);}
}
