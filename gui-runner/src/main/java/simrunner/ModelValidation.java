package simrunner;

import com.jme3.asset.AssetManager;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.*;
import com.jme3.scene.Node;
import com.jme3.system.NativeLibraryLoader;
import com.qualcomm.robotcore.hardware.HardwareMap;
import simcore.RobotUrdf;
import java.nio.file.*;
import java.util.*;

/** Native preparation check, shared with preview. Human shape review is a separate explicit step. */
public final class ModelValidation {
    static final class Prepared implements AutoCloseable {
        final PhysicsSpace space;
        final Node root=new Node("prepared model");
        final ModelProfile profile;
        Prepared(Path file,AssetManager assets)throws Exception {
            profile=new ModelProfile(file,false);space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);space.setGravity(Vector3f.ZERO);space.setAccuracy(1f/480);
            try {
                var world=new PhysicsWorld(assets,root,space);world.driveControllerEnabled=false;
                if(profile.kind.equals("field")) {
                    if(profile.legacyField())new ImportedFieldScene(new FieldPackage(profile.artifact("field")),FieldConfig.defaults().withSource(profile.artifact("field").toString()).withMode("game-pieces"),world,root,assets);
                    else new ModelFieldScene(profile,true,world,root,assets);
                } else {
                    Path urdfFile=profile.artifact("robot");var urdf=RobotUrdf.parse(urdfFile);
                    if(profile.runtime.containsKey("total_mass_kg"))urdf=urdf.withTotalMassKg(FieldPackage.num(profile.runtime,"total_mass_kg"));
                    Set<String> drives=new HashSet<>();
                    if(profile.runtime.containsKey("drive")){var d=FieldPackage.map(profile.runtime.get("drive"));drives.add(FieldPackage.str(d,"left_motor"));drives.add(FieldPackage.str(d,"right_motor"));}
                    else drives.addAll(List.of("left_front_drive","right_front_drive","left_back_drive","right_back_drive"));
                    var scene=new ImportedRobotScene(urdf,urdfFile,new HardwareMap(),assets,((Number)profile.runtime.get("vhacd_max_hulls")).intValue(),drives);
                    profile.configure(scene);scene.passiveConstruction=true;
                    if(profile.runtime.containsKey("flexible_intake"))scene.flexibleIntake=FlexibleIntakeConfig.parse(FieldPackage.map(profile.runtime.get("flexible_intake")));
                    var robot=new ArticulatedRobot(scene,world,root,new Vector3f());
                    if(scene.flexibleIntake!=null)world.installFlexibleIntake(new FlexibleIntake(world,scene,robot,scene.flexibleIntake));
                }
                root.updateGeometricState();
                if(space.countRigidBodies()==0)throw new IllegalArgumentException("Model has no physical bodies");
                // Candidate geometry only: unpowered, no gravity, short constraint sanity check.
                for(int i=0;i<24;i++)space.update(1f/480,0);
                for(var b:space.getRigidBodyList())if(!Vector3f.isValidVector(b.getPhysicsLocation()) || b.getPhysicsLocation().length()>1000 || b.isDynamic()&&(!Vector3f.isValidVector(b.getLinearVelocity())||b.getLinearVelocity().length()>100))throw new IllegalArgumentException("Native construction is unstable; revise overlapping shapes/mass/joints");
            }catch(Exception e){space.destroy();throw e;}
        }
        Map<String,Object> proof(){return Map.of("valid",true,"effective_digest",profile.digest,"native_bodies",space.countRigidBodies(),"check","unpowered construction + 24 fixed native steps; inspect openings/coverage before approval","engine","Minie 9.0.3 / model schema 1");}
        void writeProof()throws Exception{ProfileIO.save(profile.directory.resolve("validation.json"),proof());}
        public void close(){space.destroy();}
    }
    public static void main(String[] args)throws Exception {
        if(args.length!=1)throw new IllegalArgumentException("Usage: ModelValidation <profile.json>");
        NativeLibraryLoader.loadNativeLibrary("bulletjme",true);Path file=Path.of(args[0]);
        try(var p=new Prepared(file,new DesktopAssetManager(true))){p.writeProof();System.out.println("[MODEL VALIDATION] "+ProfileIO.json(p.proof()));}
        catch(Exception e){ProfileIO.save(file.toAbsolutePath().getParent().resolve("validation.json"),Map.of("valid",false,"error",e.getMessage()==null?e.toString():e.getMessage()));throw e;}
    }
}
