package simrunner;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.*;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.*;
import com.jme3.scene.Node;
import com.jme3.system.NativeLibraryLoader;
import java.nio.file.Path;
import java.util.*;

/** Hard native gates on the real prepared package, without requiring a renderer. */
public final class FieldPhysicsProbe {
    private static final float DT=1f/240;
    static void require(boolean ok,String message){if(!ok)throw new AssertionError(message);}
    static boolean finite(Vector3f p){return Float.isFinite(p.x)&&Float.isFinite(p.y)&&Float.isFinite(p.z);}
    public static void main(String[] args)throws Exception {
        if(args.length!=1)throw new IllegalArgumentException("Usage: FieldPhysicsProbe <prepared field.json>");
        NativeLibraryLoader.loadNativeLibrary("bulletjme",true);
        var field=new FieldPackage(Path.of(args[0]));
        for(String mode:List.of("field-only","game-pieces")) {
            var space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);space.getSolverInfo().setNumIterations(40);space.setAccuracy(DT);
            int[] ticks={0};space.addTickListener(new PhysicsTickListener(){public void prePhysicsTick(PhysicsSpace s,float dt){ticks[0]++;}public void physicsTick(PhysicsSpace s,float dt){}});
            try {
                Node root=new Node();var world=new PhysicsWorld(new DesktopAssetManager(true),root,space);
                long started=System.nanoTime();var scene=new ImportedFieldScene(field,new FieldConfig("imported",mode,args[0],"biobuzz",false,.6f,.15f),world,root,new DesktopAssetManager(true));
                require(world.gamePieces().size()==(mode.equals("field-only")?0:56),"Wrong piece inventory");
                root.updateGeometricState();
                for(var piece:world.gamePieces()) {
                    var bounds=(com.jme3.bounding.BoundingBox)piece.node().getWorldBound();
                    var instance=field.instances.stream().filter(i->piece.id().equals(i.get("id"))).findFirst().orElseThrow();
                    require(bounds.getCenter().distance(piece.body().getPhysicsLocation())<.0005f,"Ball visual/collider centers differ: "+piece.id());
                    float radius=Math.max(bounds.getXExtent(),Math.max(bounds.getYExtent(),bounds.getZExtent()));
                    require(Math.abs(radius-FieldPackage.num(instance,"radius_m"))<.0005,"Ball visual/collider dimensions differ: "+piece.id());
                }
                Node robot=new Node();root.attachChild(robot);world.buildChassis(robot,8.5,new Vector3f(-1.2f,.3f,1.2f));
                world.driveChassis(new physics.MecanumKinematics.ChassisVelocity(0,0,0),DT);
                for(int i=0;i<1200;i++){space.update(DT,0);check(space,scene);}
                require(Math.abs(world.getChassisPosition().y-.1)<.01,"Robot floor support wrong");
                for(var p:world.gamePieces())require(p.body().getPhysicsLocation().y>.02,"Ball fell below floor");
                int count=space.countRigidBodies();
                for(int k=0;k<3;k++){scene.reset();require(space.countRigidBodies()==count,"Reset duplicated bodies");for(int i=0;i<240;i++)space.update(DT,0);check(space,scene);}
                // Motion into the perimeter is resolved by imported CAD colliders.
                world.driveChassis(new physics.MecanumKinematics.ChassisVelocity(-1,0,0),DT);
                for(int i=0;i<600;i++)space.update(DT,0);
                require(world.getChassisPosition().x>-field.halfExtents.x+.20,"Robot escaped perimeter");
                world.driveChassis(new physics.MecanumKinematics.ChassisVelocity(0,0,0),DT);
                for(var hive:scene.hives) {
                    float before=ImportedFieldScene.jointPosition(hive);
                    // Actual finite torque drives the passive pivot toward the opposite stop.
                    for(int i=0;i<480;i++){hive.body().applyTorque(hive.axis().mult(-5));space.update(DT,0);check(space,scene);}
                    float after=ImportedFieldScene.jointPosition(hive);
                    require(Math.abs(after-before)>.15,"HIVE pivot failed to move under external torque: "+before+" -> "+after);
                }
                scene.reset();
                if(!world.gamePieces().isEmpty()) {
                    for(String type:List.of("pollen","blue_nectar","red_nectar")) {
                        var piece=world.gamePieces().stream().filter(p->p.type().equals(type)).findFirst().orElseThrow();
                        piece.body().setPhysicsLocation(new Vector3f(-.9f,.5f,-1.15f));piece.body().setLinearVelocity(new Vector3f(-.8f,0,0));
                        for(int i=0;i<480;i++)space.update(DT,0);
                        require(piece.body().getPhysicsLocation().y>.02 && piece.body().getPhysicsLocation().x>-field.halfExtents.x,"Ball fall/roll/wall response failed: "+type);
                    }
                    scene.reset();
                    for(var skin:field.instances)if(FieldPackage.str(skin,"mesh").contains("Hive_Goal_Bottom_Skin")) {
                        var pollen=world.gamePieces().stream().filter(p->p.type().equals("pollen")).findFirst().orElseThrow();
                        var frame=FieldPackage.transform(skin);var local=new Vector3f(.256f,0,.15f); // URDF (.256,-.15,0), on the cell floor.
                        var floor=frame.transformVector(local,null);pollen.body().setPhysicsLocation(floor.add(0,.3f,0));pollen.body().setLinearVelocity(Vector3f.ZERO);
                        var hive=scene.hives.stream().filter(h->h.body().getSpatial().getName().equals(skin.get("owner"))).findFirst().orElseThrow();boolean contacted=false;
                        for(int i=0;i<240;i++){space.update(DT,0);for(long manifold:space.listManifoldIds()) {
                            long a=com.jme3.bullet.collision.PersistentManifolds.getBodyAId(manifold),b=com.jme3.bullet.collision.PersistentManifolds.getBodyBId(manifold);
                            if((a==pollen.body().nativeId()&&b==hive.body().nativeId())||(b==pollen.body().nativeId()&&a==hive.body().nativeId()))
                                for(long point:com.jme3.bullet.collision.PersistentManifolds.listPointIds(manifold))if(com.jme3.bullet.collision.ManifoldPoints.getAppliedImpulse(point)>1e-6)contacted=true;
                        }}
                        require(contacted,"Pollen failed to contact CELL floor: "+skin.get("id"));
                        scene.reset();
                    }
                    // A pollen dropped through each FLOWER's real central opening must pass its top ring.
                    for(var instance:field.instances)if(FieldPackage.str(instance,"mesh").contains("Flower_Layer_C")) {
                        var p=world.gamePieces().get(0);var t=FieldPackage.transform(instance);var center=t.getTranslation();
                        p.body().setPhysicsLocation(center.add(0,.15f,0));p.body().setLinearVelocity(Vector3f.ZERO);
                        for(int i=0;i<180;i++)space.update(DT,0);
                        require(p.body().getPhysicsLocation().y<center.y-.10,"Pollen blocked at FLOWER opening");
                    }
                }
                System.out.printf(Locale.ROOT,"[FIELD PASS] mode=%s pieces=%d bodies=%d physicsSeconds=%.2f wallSeconds=%.2f%n",mode,world.gamePieces().size(),count,ticks[0]*DT,(System.nanoTime()-started)/1e9);
            }finally{space.destroy();}
        }
    }
    static void check(PhysicsSpace space,ImportedFieldScene scene) {
        for(PhysicsRigidBody body:space.getRigidBodyList())if(body.isDynamic()) {
            require(finite(body.getPhysicsLocation())&&finite(body.getLinearVelocity())&&finite(body.getAngularVelocity()),"Nonfinite native field body");
            require(body.getLinearVelocity().length()<5,"Explosive field velocity: "+body.getLinearVelocity());
        }
        for(var h:scene.hives){float q=ImportedFieldScene.jointPosition(h);require(q>=h.lower()-.05&&q<=h.upper()+.05,"HIVE exceeded stops: "+q);}
    }
}
