package simrunner;

import com.jme3.math.Vector3f;
import com.jme3.bullet.objects.PhysicsRigidBody;
import physics.TireFriction;
import simcore.SimDcMotorEx;
import simcore.RobotUrdf;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.*;

/** Ray-supported individual tire forces with shared shaft inertia and equal tire reaction torque. */
final class TireDrive {
    record Wheel(String joint, String motor, Vector3f hub, double inertia) {}
    record State(String joint, boolean supported, double surfaceMps, double hubSpeedMps, double slipMps, double forceN, boolean sliding) {}
    private final PhysicsWorld world;
    private final DifferentialDriveConfig drive;
    private final TireDriveConfig config;
    private final List<Wheel> wheels;
    private final Map<String,Side> sides=new LinkedHashMap<>();
    private final List<State> states=new ArrayList<>();
    private static final class Side {
        final SimDcMotorEx motor; final double sign, inertia;
        double omega, angle;
        Side(SimDcMotorEx motor,double sign,double inertia){this.motor=motor;this.sign=sign;this.inertia=inertia;motor.useExternalShaft();}
        void sync(){motor.syncExternalShaft(angle*sign,omega*sign);}
    }
    TireDrive(PhysicsWorld world, HardwareMap map, ImportedRobotScene scene, DifferentialDriveConfig drive, TireDriveConfig config) {
        this.world=world;this.drive=drive;this.config=config;this.wheels=List.copyOf(scene.driveWheels);
        world.chassisBody().setFriction(0); // Tires provide ground friction; avoid counting chassis-floor drag twice.
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
        for(Side s:sides.values()) {
            s.sync();
            double torque=s.motor.externalShaftTorque()*s.sign;
            s.omega+=torque*dt/(s.inertia+dt*s.motor.externalTorqueDamping());
        }
        var body=world.chassisBody();
        Vector3f forward=world.getChassisRotation().mult(Vector3f.UNIT_X);forward.y=0;forward.normalizeLocal();
        Vector3f left=new Vector3f(forward.z,0,-forward.x);
        List<Vector3f> hubs=wheels.stream().map(w->world.robotPointWorld(w.hub)).toList();
        boolean[] supported=new boolean[wheels.size()];int count=0;
        for(int i=0;i<wheels.size();i++) {
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
            if(!supported[i]||!body.isDynamic()){states.add(new State(wheel.joint,false,surface,velocity.dot(forward),slip,0,false));continue;}
            TireFriction.Spec spec=config.omniJoints().contains(wheel.joint)?config.omni():config.traction();
            double rx=offset.cross(forward).y,ry=offset.cross(left).y;
            double invLong=1/world.driveMass()+rx*rx/world.driveYawInertia()+drive.wheelRadiusM()*drive.wheelRadiusM()/side.inertia;
            double invLat=1/world.driveMass()+ry*ry/world.driveYawInertia();
            var force=TireFriction.solve(spec,slip,velocity.dot(left),world.driveMass()*9.81/count,dt,invLong,invLat);
            body.applyImpulse(forward.mult((float)(force.longitudinalN()*dt)).add(left.mult((float)(force.lateralN()*dt))),offset);
            side.omega-=force.longitudinalN()*drive.wheelRadiusM()*dt/side.inertia;
            states.add(new State(wheel.joint,true,surface,velocity.dot(forward),slip,force.longitudinalN(),force.sliding()));
        }
        for(Side s:sides.values()){s.angle+=s.omega*dt;s.sync();}
    }
    List<State> states(){return List.copyOf(states);}
}
