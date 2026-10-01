package simrunner;

import com.jme3.asset.AssetManager;
import com.jme3.bullet.*;
import com.jme3.bullet.collision.shapes.*;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.joints.HingeJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.material.Material;
import com.jme3.math.*;
import com.jme3.scene.*;
import java.util.*;
import static simrunner.FieldPackage.*;

/** Prepared, metric passive field. Convex sheet patches preserve open cell interiors. */
final class ImportedFieldScene implements PhysicsTickListener {
    final FieldPackage field;
    final Node root=new Node("imported-field");
    final List<PhysicsRigidBody> fixed=new ArrayList<>();
    record Hive(RigidBodyControl body, HingeJoint hinge, Vector3f axis, float sign, float reference,
                float lower, float upper, float springTorque, Vector3f start, Quaternion rotation) { }
    static float jointPosition(Hive h) {float angle=h.hinge().getHingeAngle()-h.reference();return (float)Math.atan2(Math.sin(angle),Math.cos(angle))*h.sign();}
    final List<Hive> hives=new ArrayList<>();
    private final Map<String,Mesh> meshCache=new HashMap<>();
    private final Map<String,CollisionShape> shapeCache=new HashMap<>();
    private final AssetManager assets;
    private final PhysicsWorld world;
    private final FieldConfig config;

    ImportedFieldScene(FieldPackage field, FieldConfig config, PhysicsWorld world, Node scene, AssetManager assets) throws Exception {
        this.field=field;this.config=config;this.world=world;this.assets=assets;
        scene.attachChild(root);
        Node floor=new Node("field-floor-collision");root.attachChild(floor);
        float floorHalf=parameter("floor_thickness_m",.02f)/2;
        var floorShape=new BoxCollisionShape(new Vector3f(field.halfExtents.x,floorHalf,field.halfExtents.z));floorShape.setMargin(parameter("floor_margin_m",.001f));
        var floorBody=new RigidBodyControl(floorShape,0);floor.addControl(floorBody);floorBody.setPhysicsLocation(new Vector3f(0,parameter("floor_top_m",0)-floorHalf,0));
        material(floorBody);world.space().add(floorBody);fixed.add(floorBody);
        for(var inst:field.instances) {
            String owner=str(inst,"owner"),cat=str(inst,"category");
            if(!owner.equals("fixed")||cat.equals("reference")||cat.equals("detail")&&!config.fullDetail())continue;
            Node node=new Node(str(inst,"id"));root.attachChild(node);node.setLocalTransform(transform(inst));node.attachChild(geometry(inst));
            var shape=shape(str(inst,"mesh"));
            if(shape!=null && !cat.equals("floor") && !cat.equals("decoration") && !cat.equals("detail")) {
                var body=new RigidBodyControl(shape,0);node.addControl(body);material(body);world.space().add(body);fixed.add(body);
            }
        }
        for(var h:field.hives) buildHive(h);
        if(config.hasPieces())for(var inst:field.instances)if(str(inst,"owner").equals("piece"))buildPiece(inst);
        world.space().addTickListener(this);
        System.out.printf(Locale.ROOT,"[FIELD] %s | %.6f x %.6f m interior | units=m scale=1 | %s | %d pieces | %d passive HIVES%n",
            field.name,field.halfExtents.x*2,field.halfExtents.z*2,config.mode(),world.gamePieces().size(),hives.size());
        for(var note:field.approximations)System.out.println("[FIELD MODEL] "+note);
    }

    private float parameter(String key,float fallback){return field.modelParameters.containsKey(key)?(float)num(field.modelParameters,key):fallback;}
    private void material(RigidBodyControl body){
        boolean dynamic=body.isDynamic();body.setFriction(parameter("friction",config.friction()));
        body.setRestitution(parameter(dynamic?"restitution":"fixed_restitution",dynamic?config.restitution():0));
        body.setRollingFriction(parameter(dynamic?"rolling_friction":"fixed_rolling_friction",dynamic?.005f:0));
        body.setSpinningFriction(parameter(dynamic?"spinning_friction":"fixed_spinning_friction",dynamic?.005f:0));
        if(Boolean.TRUE.equals(field.modelParameters.get("use_contact_compliance"))){body.setContactStiffness(parameter("contact_stiffness_n_per_m",1e30f));body.setContactDamping(parameter("contact_damping_ns_per_m",.1f));}
    }
    Geometry geometry(Map<String,Object> instance) throws Exception {
        String id=str(instance,"mesh");Mesh mesh=meshCache.get(id);
        if(mesh==null){mesh=StlMeshLoader.load(field.resolve(str(field.meshes.get(id),"file")),new double[]{1,1,1},100_000);meshCache.put(id,mesh);}
        Geometry geometry=new Geometry(str(instance,"id"),mesh);
        var rgba=vector(instance.get("rgba"),4);
        Material material=new Material(assets,"Common/MatDefs/Light/Lighting.j3md");
        material.setBoolean("UseMaterialColors",true);var color=new ColorRGBA((float)rgba[0],(float)rgba[1],(float)rgba[2],1);
        material.setColor("Diffuse",color);material.setColor("Ambient",color);material.getAdditionalRenderState().setFaceCullMode(com.jme3.material.RenderState.FaceCullMode.Off);
        geometry.setMaterial(material);return geometry;
    }

    CollisionShape shape(String name) {
        if(shapeCache.containsKey(name))return shapeCache.get(name);
        var value=field.meshes.get(name).get("collision");
        if(value==null){shapeCache.put(name,null);return null;}
        var data=map(value);var compound=new CompoundCollisionShape();
        switch(str(data,"kind")) {
            case "boxes" -> {
                var boxes=maps(data.get("boxes"));if(boxes.size()>500)throw new IllegalArgumentException("Profile box budget exceeded");
                for(var item:boxes){var size=pos(item.get("size_m"));size.set(Math.abs(size.x),Math.abs(size.y),Math.abs(size.z));var box=new BoxCollisionShape(size.mult(.5f));box.setMargin(parameter("collision_margin_m",.0005f));compound.addChildShape(box,pos(item.get("center_m")));}
            }
            case "box" -> {
                var size=pos(data.get("size_m"));size=new Vector3f(Math.abs(size.x),Math.abs(size.y),Math.abs(size.z));
                var box=new BoxCollisionShape(size.mult(.5f));box.setMargin(Math.min(parameter("collision_margin_m",.0005f),Math.min(size.x,Math.min(size.y,size.z))/8));
                compound.addChildShape(box,pos(data.get("center_m")));
            }
            case "ring" -> {
                int segments=(int)num(data,"segments");if(segments<16||segments>64)throw new IllegalArgumentException("Invalid ring segment budget");
                float inner=(float)num(data,"inner_radius_m"),outer=(float)num(data,"outer_radius_m"),bottom=(float)num(data,"bottom_m"),top=(float)num(data,"top_m");
                if(inner<=0||outer<=inner||top<=bottom)throw new IllegalArgumentException("Invalid ring");
                for(int i=0;i<segments;i++) {
                    List<Vector3f> points=new ArrayList<>();
                    for(float radius:new float[]{inner,outer})for(float y:new float[]{bottom,top})for(int k=0;k<2;k++) {
                        double a=(i+k)*Math.PI*2/segments;
                        points.add(new Vector3f(radius*(float)Math.cos(a),y,-radius*(float)Math.sin(a)));
                    }
                    var hull=new HullCollisionShape(points);hull.setMargin(parameter("collision_margin_m",.0005f));compound.addChildShape(hull);
                }
            }
            case "hulls" -> {
                if(!(data.get("hulls_m") instanceof List<?> hulls)||hulls.size()>1000)throw new IllegalArgumentException("Invalid sheet hull budget");
                for(Object h:hulls) {
                    if(!(h instanceof List<?> points)||points.size()<6||points.size()>512)throw new IllegalArgumentException("Invalid convex surface patch");
                    var hull=new HullCollisionShape(points.stream().map(FieldPackage::pos).toList());hull.setMargin(parameter("collision_margin_m",.0005f));compound.addChildShape(hull);
                }
            }
            default -> throw new IllegalArgumentException("Unknown field collision kind");
        }
        shapeCache.put(name,compound);return compound;
    }

    void buildPiece(Map<String,Object> inst) throws Exception {
        Node node=new Node(str(inst,"id"));root.attachChild(node);node.setLocalTransform(transform(inst));var geom=geometry(inst);geom.setLocalTranslation(pos(inst.get("mesh_center_m")).negate());node.attachChild(geom);
        var sphere=new SphereCollisionShape((float)num(inst,"radius_m"));
        var body=new RigidBodyControl(sphere,(float)num(inst,"mass_kg"));node.addControl(body);
        material(body);
        body.setCcdMotionThreshold(.02f);body.setCcdSweptSphereRadius((float)num(inst,"radius_m")*.8f);
        world.space().add(body);world.registerPiece(str(inst,"id"),str(inst,"category"),node,body);
    }

    private void buildHive(Map<String,Object> spec) throws Exception {
        Vector3f center=pos(spec.get("com_xyz_m"));float mass=(float)num(spec,"mass_kg");PrincipalInertia inertia=new PrincipalInertia();
        for(var sample:maps(spec.get("inertials"))) {
            var rows=(List<?>)sample.get("inertia_kg_m2");double[][] source=new double[3][];for(int i=0;i<3;i++)source[i]=vector(rows.get(i),3);
            // B I B^T for (x,z,-y).
            int[] axes={0,2,1};int[] signs={1,1,-1};double[][] tensor=new double[3][3];
            for(int a=0;a<3;a++)for(int b=0;b<3;b++)tensor[a][b]=source[axes[a]][axes[b]]*signs[a]*signs[b];
            var t=transform(sample);inertia.addRotated(tensor,t.getRotation());inertia.addParallelAxis(num(sample,"mass_kg"),t.getTranslation().subtract(center));
        }
        var principal=inertia.diagonalize(str(spec,"id"));Quaternion inverse=principal.rotation().inverse();
        Node node=new Node(str(spec,"id"));root.attachChild(node);
        var compound=new CompoundCollisionShape();
        for(var inst:field.instances) {
            if(!str(inst,"owner").equals(str(spec,"id")) || str(inst,"category").equals("detail")&&!config.fullDetail() || str(inst,"category").equals("reference"))continue;
            Transform t=transform(inst);Vector3f local=inverse.mult(t.getTranslation().subtract(center));Quaternion q=inverse.mult(t.getRotation());
            Geometry geom=geometry(inst);geom.setLocalTranslation(local);geom.setLocalRotation(q);node.attachChild(geom);
            var s=shape(str(inst,"mesh"));if(s instanceof CompoundCollisionShape children) {
                for(var child:children.listChildren())compound.addChildShape(child.getShape(),local.add(q.mult(child.copyOffset(null))),q.toRotationMatrix().mult(child.copyRotationMatrix(null)));
            }else if(s!=null)compound.addChildShape(s,local,q.toRotationMatrix());
        }
        if(compound.countChildren()==0)throw new IllegalArgumentException("HIVE requires collision skins");
        var body=new RigidBodyControl(compound,mass);node.addControl(body);body.setPhysicsLocation(center);body.setPhysicsRotation(principal.rotation());
        body.setInverseInertiaLocal(new Vector3f(1/principal.moments().x,1/principal.moments().y,1/principal.moments().z));
        body.setAngularDamping((float)num(spec,"damping"));body.setEnableSleep(false);world.space().add(body);
        Transform pivot=transform(spec);Vector3f axis=pivot.getRotation().mult(pos(spec.get("axis"))).normalizeLocal();
        var anchor=new PhysicsRigidBody(new SphereCollisionShape(.001f),0);anchor.setPhysicsLocation(pivot.getTranslation());world.space().add(anchor);fixed.add(anchor);
        var hinge=new HingeJoint(anchor,body,Vector3f.ZERO,inverse.mult(pivot.getTranslation().subtract(center)),axis,inverse.mult(axis));hinge.setCollisionBetweenLinkedBodies(false);world.space().add(hinge);
        Quaternion original=body.getPhysicsRotation();float reference=hinge.getHingeAngle();body.setPhysicsRotation(new Quaternion().fromAngleAxis(.01f,axis).mult(original));float delta=hinge.getHingeAngle()-reference;float sign=Math.signum((float)Math.atan2(Math.sin(delta),Math.cos(delta)));body.setPhysicsRotation(original);
        if(sign==0)throw new IllegalArgumentException("HIVE hinge orientation unresolved");
        float lower=(float)num(spec,"lower_rad"),upper=(float)num(spec,"upper_rad");hinge.setLimit(reference+Math.min(sign*lower,sign*upper),reference+Math.max(sign*lower,sign*upper),.9f,.3f,1);
        // Pivot brackets overlap their connected frame in CAD. Exempt adjacent structure only.
        for(var f:fixed)if(f!=anchor && f.getPhysicsLocation().distance(pivot.getTranslation())<.15f)body.addToIgnoreList(f);
        float spring=spec.containsKey("spring_peak_torque_nm")?(float)num(spec,"spring_peak_torque_nm"):1.5f;
        if(spring<0||spring>10)throw new IllegalArgumentException("HIVE spring peak torque must be 0..10 Nm");
        hives.add(new Hive(body,hinge,axis,sign,reference,lower,upper,spring,center.clone(),original));
        System.out.println("[HIVE] "+spec.get("id")+" CAD mass="+mass+"kg principal moments="+principal.moments()+" COM="+center);
    }

    void reset() {
        for(var h:hives){h.body().clearForces();h.body().setPhysicsLocation(h.start());h.body().setPhysicsRotation(h.rotation());h.body().setLinearVelocity(Vector3f.ZERO);h.body().setAngularVelocity(Vector3f.ZERO);h.body().activate();}
        world.resetPieces();
    }
    @Override public void prePhysicsTick(PhysicsSpace space,float dt) {
        // Explicit uncalibrated passive two-position spring. Finite torque, no pose writes.
        // The official HIVE is bi-stable; exporter hinge efforts are not physical damper data.
        for(var h:hives) {
            float q=jointPosition(h);
            float torque=-h.springTorque()*(float)Math.sin(2*Math.PI*(q-h.lower())/(h.upper()-h.lower()));
            h.body().applyTorque(h.axis().mult(torque));
        }
    }
    @Override public void physicsTick(PhysicsSpace space,float dt) { }
}
