package simrunner;

import com.jme3.asset.AssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.EmptyShape;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.scene.*;
import java.util.*;

/** Displays actual native shapes, including margins, compounds and flexible segments. */
final class CollisionOverlay {
    final Node root=new Node("native-collision-overlay");
    private record Entry(PhysicsRigidBody body,Spatial shape) { }
    private final Map<Long,Entry> entries=new HashMap<>();
    private final PhysicsSpace space;
    private final AssetManager assets;
    boolean visible;
    private String focused="";
    void focus(String bodyName){focused=bodyName;}
    CollisionOverlay(PhysicsSpace space,Node scene,AssetManager assets) {
        this.space=space;this.assets=assets;scene.attachChild(root);setVisible(false);
    }
    void setVisible(boolean value){visible=value;root.setCullHint(value?Spatial.CullHint.Inherit:Spatial.CullHint.Always);}
    void update() {
        if(!visible)return;
        Set<Long> present=new HashSet<>();
        for(var body:space.getRigidBodyList()) {
            if(body.getCollisionShape() instanceof EmptyShape)continue;
            present.add(body.nativeId());var entry=entries.get(body.nativeId());
            if(entry==null) {
                Spatial shape=DebugShapeFactory.getDebugShape(body.getCollisionShape());
                var material=new Material(assets,"Common/MatDefs/Misc/Unshaded.j3md");
                material.setColor("Color",body.isDynamic()?new ColorRGBA(0,1,1,1):new ColorRGBA(1,.55f,0,1));
                material.getAdditionalRenderState().setWireframe(true);
                material.getAdditionalRenderState().setDepthTest(false);
                material.getAdditionalRenderState().setDepthWrite(false);
                shape.setMaterial(material);shape.setQueueBucket(com.jme3.renderer.queue.RenderQueue.Bucket.Transparent);root.attachChild(shape);
                entry=new Entry(body,shape);entries.put(body.nativeId(),entry);
            }
            String name=body instanceof com.jme3.bullet.control.RigidBodyControl control&&control.getSpatial()!=null?control.getSpatial().getName():"";
            ColorRGBA color=!focused.isEmpty()&&name.equals("body-"+focused)?ColorRGBA.Magenta:body.isDynamic()?ColorRGBA.Cyan:new ColorRGBA(1,.55f,0,1);
            if(entry.shape() instanceof Geometry geometry)geometry.getMaterial().setColor("Color",color);else for(Geometry geometry:geometries(entry.shape()))geometry.getMaterial().setColor("Color",color);
            entry.shape().setLocalTranslation(body.getPhysicsLocation());entry.shape().setLocalRotation(body.getPhysicsRotation());
        }
        for(var iterator=entries.entrySet().iterator();iterator.hasNext();) {
            var entry=iterator.next();if(!present.contains(entry.getKey())){entry.getValue().shape().removeFromParent();iterator.remove();}
        }
    }
    private static List<Geometry> geometries(Spatial spatial){if(spatial instanceof Geometry g)return List.of(g);var out=new ArrayList<Geometry>();if(spatial instanceof Node n)for(var child:n.getChildren())out.addAll(geometries(child));return out;}
    int shapeCount(){return entries.size();}
}
