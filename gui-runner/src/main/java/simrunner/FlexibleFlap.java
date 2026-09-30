package simrunner;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.math.*;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import java.nio.FloatBuffer;
import java.util.*;

/** Two cantilever arms, discretized into collidable rigid segments with elastic bending hinges. */
final class FlexibleFlap {
    record Segment(PhysicsRigidBody body, Vector3f restCenter, New6Dof hinge) {}
    final List<Segment> segments=new ArrayList<>();
    private final FlexibleIntakeConfig spec;
    private Geometry visual;
    private java.util.function.Supplier<Transform> visualFrame;
    private float[] restPositions,restNormals;
    private Mesh deformed;
    private int[] bones;
    private Quaternion meshToBeam = new Quaternion();
    FlexibleFlap(PhysicsSpace space,PhysicsRigidBody shaft,Transform worldFrame,double mass,FlexibleIntakeConfig spec) {
        this.spec=spec;
        double length=spec.armLengthM()/spec.segmentsPerArm();
        for(int arm:new int[]{1,-1}) {
            PhysicsRigidBody parent=shaft;
            for(int i=0;i<spec.segmentsPerArm();i++) {
                Vector3f rest=new Vector3f(0,(float)(arm*(i+.5)*length),0);
                var shape=new BoxCollisionShape(new Vector3f((float)spec.thicknessM()/2,(float)length/2,(float)spec.widthM()/2));
                shape.setMargin(.0005f);
                var body=new PhysicsRigidBody(shape,(float)(mass/(2*spec.segmentsPerArm())));
                body.setPhysicsRotation(worldFrame.getRotation());
                body.setPhysicsLocation(worldFrame.transformVector(rest,null));
                body.setContactStiffness((float)spec.contactStiffnessNPerM());
                body.setContactDamping((float)spec.contactDampingNsPerM());
                body.setFriction((float)spec.friction());body.setEnableSleep(false);
                space.add(body);parent.addToIgnoreList(body);
                if (parent != shaft) shaft.addToIgnoreList(body);
                Vector3f pivot=worldFrame.transformVector(new Vector3f(0,(float)(arm*i*length),0),null);
                Vector3f axis=worldFrame.getRotation().mult(Vector3f.UNIT_Z);
                Quaternion frame=new Quaternion().fromRotationMatrix(frameForX(axis));
                New6Dof hinge=New6Dof.newInstance(parent,body,pivot,frame,RotationOrder.XYZ);
                for(int dof=0;dof<6;dof++){hinge.set(MotorParam.LowerLimit,dof,0);hinge.set(MotorParam.UpperLimit,dof,0);}
                hinge.set(MotorParam.LowerLimit,3,(float)-spec.maxBendRad());hinge.set(MotorParam.UpperLimit,3,(float)spec.maxBendRad());
                hinge.enableSpring(3,true);hinge.setStiffness(3,(float)spec.stiffnessNmPerRad(),true);
                hinge.setDamping(3,(float)spec.dampingRatio(),true);hinge.setEquilibriumPoint(3,0);
                space.add(hinge);segments.add(new Segment(body,rest,hinge));parent=body;
            }
        }
    }
    void bindVisual(Geometry geometry, java.util.function.Supplier<Transform> frame, Quaternion meshToBeam) {
        visual=geometry;visualFrame=frame;this.meshToBeam=meshToBeam.clone();
        // Each instance needs independent deformable buffers; CAD instances otherwise share meshes.
        deformed=geometry.getMesh().deepClone();deformed.setDynamic();geometry.setMesh(deformed);
        FloatBuffer p=deformed.getFloatBuffer(VertexBuffer.Type.Position);restPositions=new float[p.limit()];p.duplicate().rewind().get(restPositions);
        FloatBuffer n=deformed.getFloatBuffer(VertexBuffer.Type.Normal);restNormals=new float[n.limit()];n.duplicate().rewind().get(restNormals);
        bones=new int[restPositions.length/3];
        double length=spec.armLengthM()/spec.segmentsPerArm();
        for(int i=0;i<bones.length;i++) {
            double y=restPositions[3*i+1];int section=Math.min(spec.segmentsPerArm()-1,(int)(Math.abs(y)/length));
            bones[i]=section+(y<0?spec.segmentsPerArm():0);
        }
    }
    void updateVisual() {
        if(visual==null)return;
        var worldToVisual=visualFrame.get().invert();
        FloatBuffer p=deformed.getFloatBuffer(VertexBuffer.Type.Position),n=deformed.getFloatBuffer(VertexBuffer.Type.Normal);
        Vector3f point=new Vector3f(),normal=new Vector3f();
        // Cache bone transforms once per update, rather than crossing JNI for every CAD vertex.
        Transform[] transforms=new Transform[segments.size()];Quaternion[] normals=new Quaternion[segments.size()];
        for(int i=0;i<segments.size();i++) {
            Segment s=segments.get(i);Quaternion rotation=s.body.getPhysicsRotation();
            transforms[i]=new Transform(s.body.getPhysicsLocation().subtract(rotation.mult(s.restCenter)),rotation.mult(meshToBeam)).combineWithParent(worldToVisual);
            normals[i]=worldToVisual.getRotation().mult(rotation).mult(meshToBeam);
        }
        for(int i=0;i<bones.length;i++) {
            point.set(restPositions[3*i],restPositions[3*i+1],restPositions[3*i+2]);transforms[bones[i]].transformVector(point,point);
            p.put(3*i,point.x);p.put(3*i+1,point.y);p.put(3*i+2,point.z);
            normal.set(restNormals[3*i],restNormals[3*i+1],restNormals[3*i+2]);normals[bones[i]].mult(normal,normal);
            n.put(3*i,normal.x);n.put(3*i+1,normal.y);n.put(3*i+2,normal.z);
        }
        deformed.getBuffer(VertexBuffer.Type.Position).setUpdateNeeded();deformed.getBuffer(VertexBuffer.Type.Normal).setUpdateNeeded();deformed.updateBound();
    }
    double maxDeflectionRad(){return segments.stream().mapToDouble(s->Math.abs(s.hinge.getAngles(null).x)).max().orElse(0);}
    double mass(){return segments.stream().mapToDouble(s->s.body.getMass()).sum();}
    private static Matrix3f frameForX(Vector3f x){Vector3f ref=Math.abs(x.y)<.9?Vector3f.UNIT_Y:Vector3f.UNIT_Z;Vector3f z=x.cross(ref).normalizeLocal();return new Matrix3f().setColumn(0,x).setColumn(1,z.cross(x).normalizeLocal()).setColumn(2,z);}
}
