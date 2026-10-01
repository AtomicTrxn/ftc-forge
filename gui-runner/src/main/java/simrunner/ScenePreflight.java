package simrunner;

import com.jme3.asset.*;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.*;
import com.jme3.scene.Node;
import com.qualcomm.robotcore.hardware.HardwareMap;
import simcore.*;
import java.nio.file.*;
import java.util.*;

/** Unpowered scene construction and placement validation without TeamCode or a project. */
public final class ScenePreflight {
    static final class Prepared implements AutoCloseable {
        final PhysicsSpace space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        final Node root=new Node("scene-preflight");
        final PhysicsWorld world;
        Prepared(Path file, AssetManager assets) throws Exception {this(file,assets,true);}
        Prepared(Path file, AssetManager assets, boolean check) throws Exception {
            world=new PhysicsWorld(assets,root,space);world.driveControllerEnabled=false;space.setGravity(Vector3f.ZERO);
            try {
                var raw=MiniJson.parseObject(Files.readString(file));var config=SceneProfile.apply(file.toRealPath(),new LinkedHashMap<>());
                if(!(config.get("robot_model_profile") instanceof String robotRef))throw new IllegalArgumentException("Select a reviewed robot model before checking the scene.");
                var robot=new ModelProfile(Path.of(robotRef),true);var layout=FieldPackage.map(config.get("scene_pieces"));var f=FieldConfig.parse(FieldPackage.map(config.get("field")));
                Vector3f half=new Vector3f(1.8288f,0,1.8288f);float floorTop=0;var environment=new HashSet<Long>();
                if(f.source().equals("imported")) {
                    var field=new ModelProfile(Path.of(f.packagePath()),true);floorTop=(float)FieldPackage.num(field.parameters,"floor_top_m");
                    if(field.legacyField()) {var built=new ImportedFieldScene(new FieldPackage(field.artifact("field")),f,world,root,assets);half=built.field.halfExtents;if(f.hasPieces()){SceneProfile.legacyPieces(layout,built,world);SceneProfile.placePieces(layout,world);}for(var body:built.fixed)environment.add(body.nativeId());for(var hive:built.hives)environment.add(hive.body().nativeId());}
                    else {var built=new ModelFieldScene(field,f.hasPieces(),world,root,assets,layout);half=built.halfExtents;for(var body:built.bodies)environment.add(body.nativeId());}
                } else {world.buildFieldBoundary();if(f.hasPieces())PracticePieces.build(world,root,assets,f.usesTorus(),robot.runtime.containsKey("flexible_intake"),layout,f.friction(),f.restitution());root.getChild("floor").setCullHint(com.jme3.scene.Spatial.CullHint.Inherit);}
                Path urdfFile=robot.artifact("robot");var urdf=RobotUrdf.parse(urdfFile);
                if(robot.runtime.containsKey("total_mass_kg"))urdf=urdf.withTotalMassKg(FieldPackage.num(robot.runtime,"total_mass_kg"));
                Set<String> drives=robot.runtime.containsKey("drive")?Set.copyOf(DifferentialDriveConfig.parse(FieldPackage.map(robot.runtime.get("drive"))).motorNames()):Set.of("left_front_drive","right_front_drive","left_back_drive","right_back_drive");
                var scene=new ImportedRobotScene(urdf,urdfFile,new HardwareMap(),assets,((Number)robot.runtime.get("vhacd_max_hulls")).intValue(),drives);robot.configure(scene);scene.passiveConstruction=true;
                if(robot.runtime.containsKey("flexible_intake"))scene.flexibleIntake=FlexibleIntakeConfig.parse(FieldPackage.map(robot.runtime.get("flexible_intake")));
                var start=FieldPackage.pos(raw.get("robot_start_xyz_m"));float yaw=(float)FieldPackage.num(raw,"robot_start_yaw_rad");
                if(check)SceneChecks.robotStart(scene,start,yaw,half,floorTop,f.source().equals("imported")&&new ModelProfile(Path.of(f.packagePath()),true).legacyField());
                var built=new ArticulatedRobot(scene,world,root,start,Map.of(),new Quaternion().fromAngleAxis(yaw,Vector3f.UNIT_Y));
                if(scene.flexibleIntake!=null)world.installFlexibleIntake(new FlexibleIntake(world,scene,built,scene.flexibleIntake));
                root.updateGeometricState();if(check)SceneChecks.contacts(world,environment,half,floorTop);
            } catch(Exception e){space.destroy();throw e;}
        }
        public void close(){space.destroy();}
    }
    public static void main(String[] args) throws Exception {
        if(args.length!=1)throw new IllegalArgumentException("Usage: ScenePreflight <scene.json>");
        com.jme3.system.NativeLibraryLoader.loadNativeLibrary("bulletjme",true);
        try(var scene=new Prepared(Path.of(args[0]),new DesktopAssetManager(true))){System.out.println("[SCENE CHECK] Valid placement; "+scene.space.countRigidBodies()+" native bodies. TeamCode was not started.");}
    }
}
