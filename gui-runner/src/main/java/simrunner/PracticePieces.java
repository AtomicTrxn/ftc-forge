package simrunner;

import com.jme3.asset.AssetManager;
import com.jme3.scene.*;
import com.jme3.math.*;
import com.jme3.material.Material;
import com.jme3.bullet.control.RigidBodyControl;
import java.util.*;

/** Generic practice pieces and scene instances shared by preflight and runtime. */
final class PracticePieces {
    static Map<String,Object> layout(boolean torus, boolean flexible) {
        var out=new LinkedHashMap<String,Object>();
        if(torus)out.put("practice-torus",Map.of("source_id","practice-torus","enabled",true,"xyz_m",List.of(.8,0.,flexible?.034:.05),"rpy_rad",List.of(flexible?Math.PI/2:0.,0.,0.)));
        else for(int i=0;i<6;i++){double radius=i<2?.03556:.04597;String type=i<2?"pollen":i<4?"red_nectar":"blue_nectar";String id="practice-"+type+"-"+i;out.put(id,Map.of("source_id",id,"enabled",true,"xyz_m",List.of(.8,.6-i*.24,radius+.002),"rpy_rad",List.of(0.,0.,0.)));}
        out.replaceAll((key,value)->new LinkedHashMap<>(FieldPackage.map(value)));return out;
    }
    static void build(PhysicsWorld world, Node root, AssetManager assets, boolean torus, boolean flexible, Map<String,Object> layout, float friction, float restitution) {
        var sources=layout(torus,flexible);var placements=layout.isEmpty()?sources:layout;
        for(var entry:placements.entrySet()){
            var row=FieldPackage.map(entry.getValue());if(Boolean.FALSE.equals(row.get("enabled")))continue;
            String source=row.getOrDefault("source_id",entry.getKey()).toString();if(!sources.containsKey(source))throw new IllegalArgumentException("Missing practice piece template: "+source+". Choose a piece from this mode.");
            var pose=FieldPackage.transform(row);
            if(torus){world.buildGamePiece(entry.getKey(),pose.getTranslation());var piece=world.gamePieces().stream().filter(p->p.id().equals(entry.getKey())).findFirst().orElseThrow();piece.body().setPhysicsRotation(pose.getRotation());world.rememberPieceStart(piece.id());continue;}
            int index=Integer.parseInt(source.substring(source.lastIndexOf('-')+1));float radius=index<2?.03556f:.04597f;String type=index<2?"pollen":index<4?"red_nectar":"blue_nectar";
            Node node=new Node(entry.getKey());var geom=new Geometry(entry.getKey(),new com.jme3.scene.shape.Sphere(12,24,radius));var material=new Material(assets,"Common/MatDefs/Misc/Unshaded.j3md");material.setColor("Color",index<2?ColorRGBA.Yellow:index<4?ColorRGBA.Red:ColorRGBA.Blue);geom.setMaterial(material);node.attachChild(geom);root.attachChild(node);
            var body=new RigidBodyControl(new com.jme3.bullet.collision.shapes.SphereCollisionShape(radius),index<2?.0209836f:.0405855f);node.addControl(body);body.setPhysicsLocation(pose.getTranslation());body.setPhysicsRotation(pose.getRotation());body.setFriction(friction);body.setRestitution(restitution);body.setRollingFriction(.005f);world.space().add(body);world.registerPiece(entry.getKey(),type,node,body);
        }
    }
}
