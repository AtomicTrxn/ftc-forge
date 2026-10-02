package simrunner;

import com.jme3.asset.AssetManager;
import com.jme3.math.*;
import com.jme3.scene.Node;
import com.jme3.bullet.control.RigidBodyControl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import simcore.RobotUrdf;
import java.util.*;

/** Generic static field assembly, passive mechanisms, and independently rolling piece assemblies. */
final class ModelFieldScene {
    final ModelProfile profile;
    final Vector3f halfExtents;
    final List<RigidBodyControl> bodies=new ArrayList<>();
    private record Start(RigidBodyControl body,Vector3f position,Quaternion rotation){}
    private final List<Start> starts=new ArrayList<>();
    ModelFieldScene(ModelProfile p,boolean pieces,PhysicsWorld primary,Node root,AssetManager assets) throws Exception {
        this(p,pieces,primary,root,assets,Map.of());
    }
    ModelFieldScene(ModelProfile p,boolean pieces,PhysicsWorld primary,Node root,AssetManager assets,Map<String,Object> layout)throws Exception {
        profile=p;
        if(pieces&&!layout.isEmpty()){var sources=new HashSet<String>();for(var part:FieldPackage.maps(FieldPackage.map(p.receipt.get("artifacts")).get("pieces")))sources.add(FieldPackage.str(part,"id"));for(var entry:layout.entrySet()){var value=FieldPackage.map(entry.getValue());if(!Boolean.FALSE.equals(value.get("enabled"))&&!sources.contains(value.getOrDefault("source_id",entry.getKey()).toString()))throw new IllegalArgumentException("Missing scene piece template: "+entry.getKey()+". Open the scene editor to select a replacement.");}}
        var h=FieldPackage.vector(p.parameters.get("field_half_extents_m"),2);halfExtents=new Vector3f((float)h[0],0,(float)h[1]);
        var structure=RobotUrdf.parse(p.artifact("field"));
        if(structure.links.values().stream().allMatch(link->link.massKg()==0&&link.collisions().isEmpty())){
            var visual=new ImportedRobotScene(structure,p.artifact("field"),new HardwareMap(),assets,8,Set.of());root.attachChild(visual.root);
        }else build(p.artifact("field"),new Vector3f(),new Quaternion(),true,primary,root,assets);
        if("generated".equals(p.parameters.get("floor_strategy"))) {
            float thickness=(float)FieldPackage.num(p.parameters,"floor_thickness_m"),top=(float)FieldPackage.num(p.parameters,"floor_top_m");
            var node=new Node("generated preparation floor");var shape=new com.jme3.bullet.collision.shapes.BoxCollisionShape(new Vector3f(halfExtents.x,thickness/2,halfExtents.z));shape.setMargin((float)FieldPackage.num(p.parameters,"floor_margin_m"));
            var geom=new com.jme3.scene.Geometry(node.getName(),new com.jme3.scene.shape.Box(halfExtents.x,thickness/2,halfExtents.z));var material=new com.jme3.material.Material(assets,"Common/MatDefs/Misc/Unshaded.j3md");material.setColor("Color",ColorRGBA.DarkGray);geom.setMaterial(material);node.attachChild(geom);root.attachChild(node);
            var floor=new RigidBodyControl(shape,0);node.addControl(floor);floor.setPhysicsLocation(new Vector3f(0,top-thickness/2,0));floor.setFriction((float)FieldPackage.num(p.parameters,"friction"));floor.setRestitution((float)FieldPackage.num(p.parameters,"fixed_restitution"));floor.setRollingFriction((float)FieldPackage.num(p.parameters,"fixed_rolling_friction"));floor.setSpinningFriction((float)FieldPackage.num(p.parameters,"fixed_spinning_friction"));primary.space().add(floor);bodies.add(floor);
        }
        if(pieces)for(var piece:FieldPackage.maps(FieldPackage.map(p.receipt.get("artifacts")).get("pieces"))) {
            String sourceId=FieldPackage.str(piece,"id");var instances=new LinkedHashMap<String,Map<String,Object>>();
            if(layout.isEmpty())instances.put(sourceId,piece);
            else for(var entry:layout.entrySet()){var value=FieldPackage.map(entry.getValue());if(sourceId.equals(value.getOrDefault("source_id",entry.getKey()))&&!Boolean.FALSE.equals(value.get("enabled")))instances.put(entry.getKey(),value);}
            for(var instance:instances.entrySet()){
                var placement=FieldPackage.transform(instance.getValue());var robot=build(p.resolve(FieldPackage.str(piece,"file")),placement.getTranslation(),placement.getRotation(),false,primary,root,assets);
                var body=robot.bodyForLink(robotRoot(piece,p));primary.registerPiece(instance.getKey(),FieldPackage.str(piece,"type"),robot.chassisNode,body);
            }
        }
        System.out.println("[FIELD MODEL] "+p.name+" | "+halfExtents.x*2+" x "+halfExtents.z*2+" m | "+bodies.size()+" bodies");
    }
    private String robotRoot(Map<String,Object> piece,ModelProfile p)throws Exception{return RobotUrdf.parse(p.resolve(FieldPackage.str(piece,"file"))).rootLink;}
    private ArticulatedRobot build(java.nio.file.Path file,Vector3f position,Quaternion rotation,boolean fixed,PhysicsWorld primary,Node root,AssetManager assets)throws Exception {
        var urdf=RobotUrdf.parse(file);var scene=new ImportedRobotScene(urdf,file,new HardwareMap(),assets,((Number)profile.runtime.get("vhacd_max_hulls")).intValue(),Set.of());
        profile.configure(scene);scene.passiveConstruction=true;scene.lockChassisLevel=false;
        var before=new HashSet<>(primary.space().getRigidBodyList());var local=new PhysicsWorld(assets,root,primary.space());local.driveControllerEnabled=false;
        var robot=new ArticulatedRobot(scene,local,root,position,Map.of(),rotation);
        if(fixed)robot.bodyForLink(urdf.rootLink).setMass(0);
        for(var body:primary.space().getRigidBodyList())if(!before.contains(body)&&body instanceof RigidBodyControl control){bodies.add(control);starts.add(new Start(control,control.getPhysicsLocation(),control.getPhysicsRotation()));}
        return robot;
    }
    void reset(){for(var s:starts){s.body.clearForces();s.body.setPhysicsLocation(s.position);s.body.setPhysicsRotation(s.rotation);if(s.body.isDynamic()){s.body.setLinearVelocity(Vector3f.ZERO);s.body.setAngularVelocity(Vector3f.ZERO);s.body.activate();}}}
}
