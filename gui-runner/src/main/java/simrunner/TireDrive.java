package simrunner;

import com.jme3.math.Vector3f;
import com.jme3.bullet.objects.PhysicsRigidBody;
import physics.TireFriction;
import simcore.SimDcMotorEx;
import simcore.RobotUrdf;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.*;

/** Contact-supported individual tire forces with shared shaft inertia and equal tire reaction torque. */
final class TireDrive {
    record Wheel(String joint, String motor, Vector3f hub, double inertia) {}
    record State(String joint, boolean supported, double surfaceMps, double hubSpeedMps, double slipMps, double forceN, boolean sliding,double normalN,double gripN,double lateralN) {}
    record Reaction(String motor,double contactTorqueNm,double shaftTorqueNm) { }
    private final PhysicsWorld world;
    private final DifferentialDriveConfig drive;
    private final TireDriveConfig config;
    private final List<Wheel> wheels;
    private final Map<String,Side> sides=new LinkedHashMap<>();
    private final List<State> states=new ArrayList<>();
    private final List<Reaction> reactions=new ArrayList<>();
    private static final class Side {
        final SimDcMotorEx motor; final double sign, inertia;
        double omega, angle;
        Side(SimDcMotorEx motor,double sign,double inertia){this.motor=motor;this.sign=sign;this.inertia=inertia;motor.useExternalShaft();}
        void sync(){motor.syncExternalShaft(angle*sign,omega*sign);}
    }
    TireDrive(PhysicsWorld world, HardwareMap map, ImportedRobotScene scene, DifferentialDriveConfig drive, TireDriveConfig config) {
        this.world=world;this.drive=drive;this.config=config;this.wheels=List.copyOf(scene.driveWheels);
        if(world.driveContacts()==null)world.chassisBody().setFriction(0); // Legacy ray support uses a frictionless chassis proxy.
        if(wheels.isEmpty())throw new IllegalArgumentException("Tire model needs imported continuous drive-wheel joints");
        for(String joint:config.omniJoints())if(wheels.stream().noneMatch(w->w.joint.equals(joint)))
            throw new IllegalArgumentException("Unknown omni tire joint: "+joint);
        for(String motor:drive.motorNames()) {
            double inertia=config.reflectedMotorInertiaKgM2()+wheels.stream().filter(w->w.motor.equals(motor)).mapToDouble(Wheel::inertia).sum();
            if(wheels.stream().noneMatch(w->w.motor.equals(motor)))throw new IllegalArgumentException("No tire joint for drive motor "+motor);
            sides.put(motor,new Side(map.get(SimDcMotorEx.class,motor),motor.equals(drive.leftMotor())?drive.leftShaftSign():drive.rightShaftSign(),inertia));
        }
    }
    /** Snapshot while the original CAD tree is intact. */
    static List<Wheel> layout(ImportedRobotScene scene) {
        List<Wheel> result=new ArrayList<>();
        for(var joint:scene.urdf.joints.values()) {
            if(!scene.isDriveWheel(joint))continue;
            var actuators=scene.urdf.transmissions.values().stream().filter(t->t.joint().equals(joint.name())).flatMap(t->t.actuators().stream()).toList();
            if(!joint.type().equals("continuous")||actuators.size()!=1||Math.abs(actuators.get(0).mechanicalReduction())!=1)
                throw new IllegalArgumentException("Tires require continuous wheels coupled 1:1 to one side motor");
            var mount=scene.linkNodes.get(joint.child());
            Vector3f pivot=mount.getWorldTranslation().clone();
            Vector3f axis=mount.getWorldRotation().mult(ImportedRobotScene.position(joint.axis())).normalizeLocal();
            if(Math.abs(axis.dot(Vector3f.UNIT_Z))<.999)
                throw new IllegalArgumentException("Differential tire axes must run across the chassis");
            Set<String> branch=new HashSet<>(); collect(joint.child(),scene.urdf,branch);
            double inertia=0;
            for(String name:branch) {
                var link=scene.urdf.links.get(name);var node=scene.linkNodes.get(name);
                Vector3f center=node.localToWorld(ImportedRobotScene.position(link.inertialOrigin().xyz()),null).subtract(pivot);
                Vector3f a=node.getWorldRotation().mult(ImportedRobotScene.rotation(link.inertialOrigin().rpy())).inverse().mult(axis);
                var i=link.inertia();
                inertia+=i.ixx()*a.x*a.x+i.izz()*a.y*a.y+i.iyy()*a.z*a.z
                    +2*i.ixz()*a.x*a.y-2*i.ixy()*a.x*a.z-2*i.iyz()*a.y*a.z
                    +link.massKg()*(center.lengthSquared()-Math.pow(center.dot(axis),2));
            }
            result.add(new Wheel(joint.name(),actuators.get(0).name(),pivot,Math.max(0,inertia)));
        }
        return result;
    }
    private static void collect(String name,RobotUrdf urdf,Set<String> result){result.add(name);for(var j:urdf.joints.values())if(j.parent().equals(name))collect(j.child(),urdf,result);}
    void tick(float dt) {
        if(dt<=0)return;
        var beforeContact=new LinkedHashMap<String,Double>();
        for(var entry:sides.entrySet()) {Side s=entry.getValue();
            s.sync();
            double torque=s.motor.externalShaftTorque()*s.sign;
            s.omega+=torque*dt/(s.inertia+dt*s.motor.externalTorqueDamping());
            beforeContact.put(entry.getKey(),s.omega);
        }
        var body=world.chassisBody();
        Vector3f forward=world.getChassisRotation().mult(Vector3f.UNIT_X);forward.y=0;forward.normalizeLocal();
        Vector3f left=new Vector3f(forward.z,0,-forward.x);
        List<Vector3f> hubs=wheels.stream().map(w->world.robotPointWorld(w.hub)).toList();
        boolean[] supported=new boolean[wheels.size()];int count=0;
        for(int i=0;i<wheels.size();i++) {
            if(world.driveContacts()!=null){supported[i]=world.driveContacts().snapshot().supported(wheels.get(i).joint);if(supported[i])count++;continue;}
            Vector3f hub=hubs.get(i),end=hub.add(0,(float)(-drive.wheelRadiusM()-config.contactToleranceM()),0);
            for(var hit:world.space().rayTest(hub,end)) {
                // Current FTC floor model is flat and static. Ignore every robot/game-piece body.
                if(hit.getCollisionObject() instanceof PhysicsRigidBody ground && ground != body && ground.isStatic()
                    && hit.getHitNormalLocal().y>.7f){supported[i]=true;count++;break;}
            }
        }
        states.clear();
        for(int i=0;i<wheels.size();i++) {
            Wheel wheel=wheels.get(i);Side side=sides.get(wheel.motor);
            Vector3f hub=hubs.get(i),point=hub.add(0,(float)-drive.wheelRadiusM(),0);
            Vector3f offset=point.subtract(body.getPhysicsLocation());
            Vector3f velocity=body.getLinearVelocity().add(body.getAngularVelocity().cross(offset));
            double surface=side.omega*drive.wheelRadiusM();
            double slip=surface-velocity.dot(forward);
            if(!supported[i]||!body.isDynamic()){states.add(new State(wheel.joint,false,surface,velocity.dot(forward),slip,0,false,0,0,0));continue;}
            TireFriction.Spec spec=config.omniJoints().contains(wheel.joint)?config.omni():config.traction();
            double rx=offset.cross(forward).y,ry=offset.cross(left).y;
            double invLong=1/world.driveMass()+rx*rx/world.driveYawInertia()+drive.wheelRadiusM()*drive.wheelRadiusM()/side.inertia;
            double invLat=1/world.driveMass()+ry*ry/world.driveYawInertia();
            double normal=world.driveContacts()==null?world.driveMass()*9.81/count:world.driveContacts().snapshot().wheels().stream().filter(w->w.joint().equals(wheel.joint)).mapToDouble(DriveContacts.Support::normalImpulse).sum()/dt;
            var force=TireFriction.solve(spec,slip,velocity.dot(left),normal,dt,invLong,invLat);
            double longitudinal=force.longitudinalN(),lateral=force.lateralN();boolean sliding=force.sliding();
            double gripLimit=spec.staticMu()*normal;
            if(world.driveContacts()!=null) {
                double grip=world.driveContacts().snapshot().wheels().stream().filter(w->w.joint().equals(wheel.joint)).mapToDouble(DriveContacts.Support::frictionImpulse).sum()/dt;
                gripLimit=Math.min(gripLimit,grip);
                double requested=Math.hypot(longitudinal,lateral);
                if(requested>grip){double scale=grip/requested;longitudinal*=scale;lateral*=scale;sliding=true;}
            }
            body.applyImpulse(forward.mult((float)(longitudinal*dt)).add(left.mult((float)(lateral*dt))),offset);
            side.omega-=longitudinal*drive.wheelRadiusM()*dt/side.inertia;
            states.add(new State(wheel.joint,true,surface,velocity.dot(forward),slip,longitudinal,sliding,normal,gripLimit,lateral));
        }
        reactions.clear();
        for(var entry:sides.entrySet()){Side s=entry.getValue();double contact=states.stream().filter(w->wheels.stream().anyMatch(layout->layout.joint.equals(w.joint)&&layout.motor.equals(entry.getKey()))).mapToDouble(State::forceN).sum()*drive.wheelRadiusM();
            reactions.add(new Reaction(entry.getKey(),contact,(s.omega-beforeContact.get(entry.getKey()))*s.inertia/dt));s.angle+=s.omega*dt;s.sync();}
    }
    List<State> states(){return List.copyOf(states);}
    List<Reaction> reactions(){return List.copyOf(reactions);}
}
