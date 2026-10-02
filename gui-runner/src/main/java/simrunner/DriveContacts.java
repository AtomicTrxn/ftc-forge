package simrunner;

import com.jme3.bullet.collision.*;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;
import java.util.*;

/** Native normal support with tangential wheel-ground response owned by the drive solver. */
final class DriveContacts implements ContactListener {
    record Support(String joint,double normalImpulse,double frictionImpulse,double momentImpulse) { }
    record Snapshot(List<Support> wheels,int scrapingContacts) {
        double normalImpulse(){return wheels.stream().mapToDouble(Support::normalImpulse).sum();}
        double frictionImpulse(){return wheels.stream().mapToDouble(Support::frictionImpulse).sum();}
        double momentImpulse(){return wheels.stream().mapToDouble(Support::momentImpulse).sum();}
        boolean supported(String joint){return wheels.stream().anyMatch(w->w.joint().equals(joint));}
        String diagnosis(){return wheels.isEmpty()?"No native drive-wheel support. Check wheel collision shapes, axes, floor height and chassis clearance.":scrapingContacts>0?"Chassis also contacts the support surface; inspect belly clearance and scraping friction.":"Native drive-wheel support is present; inspect bindings, material grip and motor effort if travel is incorrect.";}
    }
    private final PhysicsWorld world;
    final DriveContactConfig config;
    private final Map<Integer,String> children;
    private final Map<String,String> joints;
    private Snapshot snapshot=new Snapshot(List.of(),0);
    DriveContacts(PhysicsWorld world,DriveContactConfig config,Map<Integer,String> children,Map<String,String> joints) {
        this.world=world;this.config=config;this.children=Map.copyOf(children);this.joints=Map.copyOf(joints);
        world.space().addContactListener(this,true,true,true);
    }
    private boolean upward(PhysicsCollisionObject a,PhysicsCollisionObject b,long point) {
        var body=world.chassisBody();boolean first=a==body;
        if(!first&&b!=body)return false;
        var other=first?b:a;
        if(!(other instanceof PhysicsRigidBody ground)||!ground.isStatic())return false;
        var normal=new Vector3f();ManifoldPoints.getNormalWorldOnB(point,normal);
        return (first?normal.y:-normal.y)>=config.minSupportNormalY()&&ManifoldPoints.getDistance1(point)<=config.maxContactGapM();
    }
    private String link(PhysicsCollisionObject a,long point) {return children.get(a==world.chassisBody()?ManifoldPoints.getIndex0(point):ManifoldPoints.getIndex1(point));}
    private static float friction(float value){return Math.max(-10,Math.min(10,value));}
    private void ownFriction(PhysicsCollisionObject a,PhysicsCollisionObject b,long point) {
        if(a!=world.chassisBody()&&b!=world.chassisBody())return;
        String name=link(a,point);
        if(name==null||!joints.containsKey(name))return;
        boolean supported=upward(a,b,point);
        // Restore Bullet's material combination when a persistent wheel contact changes surface/normal.
        // Only upward static support transfers tangential response to our drive solver.
        ManifoldPoints.setCombinedFriction(point,supported?0:friction(a.getFriction()*b.getFriction()));
        ManifoldPoints.setCombinedRollingFriction(point,supported?0:friction(a.getRollingFriction()*b.getFriction()+b.getRollingFriction()*a.getFriction()));
        ManifoldPoints.setCombinedSpinningFriction(point,supported?0:friction(a.getSpinningFriction()*b.getFriction()+b.getSpinningFriction()*a.getFriction()));
    }
    public void onContactStarted(long manifold) {
        var a=PhysicsCollisionObject.findInstance(PersistentManifolds.getBodyAId(manifold));var b=PhysicsCollisionObject.findInstance(PersistentManifolds.getBodyBId(manifold));
        for(long point:PersistentManifolds.listPointIds(manifold))ownFriction(a,b,point);
    }
    public void onContactProcessed(PhysicsCollisionObject a,PhysicsCollisionObject b,long point){ownFriction(a,b,point);}
    public void onContactEnded(long manifold){ }
    Snapshot refresh() {
        var normalByJoint=new LinkedHashMap<String,Double>();var frictionByJoint=new LinkedHashMap<String,Double>();var momentByJoint=new LinkedHashMap<String,Double>();int scraping=0;
        for(long manifold:world.space().listManifoldIds()) {
            var a=PhysicsCollisionObject.findInstance(PersistentManifolds.getBodyAId(manifold));var b=PhysicsCollisionObject.findInstance(PersistentManifolds.getBodyBId(manifold));
            if(a!=world.chassisBody()&&b!=world.chassisBody())continue;
            for(long point:PersistentManifolds.listPointIds(manifold)) {
                if(!upward(a,b,point))continue;String name=link(a,point),joint=name==null?null:joints.get(name);
                if(joint==null){scraping++;continue;}
                ownFriction(a,b,point);double normal=Math.max(0,ManifoldPoints.getAppliedImpulse(point));
                var other=a==world.chassisBody()?b:a;double mu=world.chassisBody().getFriction();
                double impulse=normal*Math.min(10,Math.max(0,mu*other.getFriction()));
                var position=new Vector3f();if(a==world.chassisBody())ManifoldPoints.getPositionWorldOnA(point,position);else ManifoldPoints.getPositionWorldOnB(point,position);
                var offset=position.subtract(world.chassisBody().getPhysicsLocation());double radius=Math.hypot(offset.x,offset.z);
                normalByJoint.merge(joint,normal,Double::sum);frictionByJoint.merge(joint,impulse,Double::sum);momentByJoint.merge(joint,impulse*radius,Double::sum);
            }
        }
        var wheels=new ArrayList<Support>();normalByJoint.forEach((joint,normal)->wheels.add(new Support(joint,normal,frictionByJoint.get(joint),momentByJoint.get(joint))));
        snapshot=new Snapshot(List.copyOf(wheels),scraping);return snapshot;
    }
    Snapshot snapshot(){return snapshot;}
}
