package simrunner;

import com.jme3.math.*;
import com.jme3.scene.Geometry;
import java.util.*;

/** Elastic collidable paddles with optional, explicitly configured compliant pinch retention. */
final class FlexibleIntake {
    private final PhysicsWorld world;
    private final FlexibleIntakeConfig config;
    TorusRetention retention;
    final List<FlexibleFlap> flaps = new ArrayList<>();
    FlexibleIntake(PhysicsWorld world, ImportedRobotScene scene, ArticulatedRobot robot, FlexibleIntakeConfig config) {
        this.world=world;this.config=config;
        world.space().getSolverInfo().setNumIterations(100);
        if (scene.flexibleIntake != config) throw new IllegalArgumentException("Configure flexible mass before constructing robot bodies");
        float yawInertia=0;
        for(String name:config.links()) {
            var link=scene.urdf.links.get(name);
            if(link==null || link.massKg()<=0)throw new IllegalArgumentException("Flexible link needs positive mass: "+name);
            String owner=scene.owners.get(name);
            if(owner.equals(scene.urdf.rootLink) || scene.wheelLinks.contains(name))
                throw new IllegalArgumentException("Flexible flap must belong to an articulated intake shaft: "+name);
            Geometry visual=null;
            for(int i=0;i<link.visuals().size();i++) {
                var g=link.visuals().get(i).geometry();
                Integer chosen=config.visualIndices().get(name);
                boolean named=java.nio.file.Path.of(g.meshFile()==null?"":g.meshFile()).getFileName().toString().equalsIgnoreCase("Flap.stl");
                boolean sole=link.visuals().stream().filter(v->v.geometry().kind().equals("mesh")).count()==1;
                if(g.kind().equals("mesh")&&(chosen!=null?chosen==i:named||sole)) {
                    if(visual!=null)throw new IllegalArgumentException("Ambiguous rubber visual; choose flexible_intake.visual_indices: "+name);
                    visual=scene.visuals.get(name).get(i);
                }
            }
            if(visual==null)throw new IllegalArgumentException("Select a flexible_intake.visual_indices mesh for link: "+name);
            Transform frame=visual.getLocalTransform().clone().combineWithParent(robot.linkFrameWorld(name));
            // The export's Flap.stl X axis is across the shaft; Z is radial. Map its
            // jME X(width), Y(radial), Z(tangent) into our beam Z(width), Y(radial), X(thickness).
            Quaternion beamToMesh=config.meshToBeam();
            frame.setRotation(frame.getRotation().mult(beamToMesh));
            var joint=scene.urdf.joints.values().stream().filter(j->j.child().equals(owner)).findFirst().orElseThrow();
            Vector3f axle=robot.linkFrameWorld(joint.parent()).getRotation().mult(ImportedRobotScene.rotation(joint.origin().rpy()))
                .mult(ImportedRobotScene.position(joint.axis())).normalizeLocal();
            if(Math.abs(axle.dot(frame.getRotation().mult(Vector3f.UNIT_Z)))<.999f)
                throw new IllegalArgumentException("Flap mesh width must align with the intake shaft: "+name);
            var flap=new FlexibleFlap(world.space(),robot.bodyForLink(name),frame,link.massKg()*config.flexMassFraction(),config);
            Geometry rubber=visual;
            flap.bindVisual(rubber, () -> rubber.getLocalTransform().clone().combineWithParent(robot.linkFrameWorld(name)),beamToMesh.inverse());flaps.add(flap);
            double shaftMoment=0;
            // Flap width axis is the shaft/bending axis. Include this attached inertia in the
            // actuator's implicit damping and safety bound, including through the chain follower.
            Vector3f shaftAxis=frame.getRotation().mult(Vector3f.UNIT_Z);
            Vector3f shaftCenter=robot.bodyForLink(name).getPhysicsLocation();
            for(var segment:flap.segments) {
                Vector3f offset=segment.body().getPhysicsLocation().subtract(world.getChassisPosition());
                Vector3f axis=segment.body().getPhysicsRotation().inverse().mult(Vector3f.UNIT_Y);
                Vector3f inverse=segment.body().getInverseInertiaLocal(null);
                Vector3f shaftOffset=segment.body().getPhysicsLocation().subtract(shaftCenter);
                Vector3f localAxis=segment.body().getPhysicsRotation().inverse().mult(shaftAxis);
                shaftMoment+=segment.body().getMass()*(shaftOffset.lengthSquared()-Math.pow(shaftOffset.dot(shaftAxis),2))
                    +localAxis.x*localAxis.x/inverse.x+localAxis.y*localAxis.y/inverse.y+localAxis.z*localAxis.z/inverse.z;
                yawInertia+=segment.body().getMass()*(offset.x*offset.x+offset.z*offset.z)
                    +axis.x*axis.x/inverse.x+axis.y*axis.y/inverse.y+axis.z*axis.z/inverse.z;
            }
            robot.addAttachedInertia(owner,shaftMoment);
        }
        world.addFlexibleYawInertia(yawInertia);
        System.out.println("[FLEX] flaps="+flaps.size()+" segments="+flaps.stream().mapToInt(f->f.segments.size()).sum()+" transferredMassKg="+mass());
    }
    void updateVisual(){for(var flap:flaps)flap.updateVisual();}
    double maxDeflectionRad(){return flaps.stream().mapToDouble(FlexibleFlap::maxDeflectionRad).max().orElse(0);}
    double mass(){return flaps.stream().mapToDouble(FlexibleFlap::mass).sum();}
    void installRetention(ArticulatedRobot robot, com.qualcomm.robotcore.hardware.HardwareMap map, MotorIntakeConfig intake, TorusRetentionConfig spec) {
        var ids=new java.util.HashSet<Long>();
        for(var flap:flaps) for(var segment:flap.segments) ids.add(segment.body().nativeId());
        retention=new TorusRetention(world,robot,map.get(simcore.SimDcMotorEx.class,intake.motor()),intake,spec,ids);
    }
    boolean contains(Vector3f point) {
        if(retention!=null && !retention.retained()) return false;
        Vector3f local=world.getChassisRotation().inverse().mult(point.subtract(world.robotPointWorld(Vector3f.ZERO)));
        Vector3f min=config.containmentMin(),max=config.containmentMax();
        return local.x>=min.x && local.x<=max.x && local.y>=min.y && local.y<=max.y && local.z>=min.z && local.z<=max.z;
    }
}
