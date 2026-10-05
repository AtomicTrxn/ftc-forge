package simrunner;

import com.jme3.bullet.collision.*;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import simcore.SimDcMotorEx;
import java.util.*;

/** Immutable, read-only view of native state; never refreshes contacts or samples/unrolls joints. */
final class PhysicsDiagnostics {
    static final int MAX_CONTACTS=64,MAX_ITEMS=32,MAX_MANIFOLDS=4096;
    record Vec(double x,double y,double z) {
        static Vec of(Vector3f v){return new Vec(v.x,v.y,v.z);}
        Vector3f vector(){return new Vector3f((float)x,(float)y,(float)z);}
    }
    record Contact(String body,String other,Vec position,Vec normal,double loadN,double gapM,boolean staticSupport,double gripN) { }
    record Wheel(String joint,Vec hub,boolean supported,double loadN,double gripN,Double slipMps,Double forceN,Double lateralN,boolean sliding) { }
    record Motor(String name,double command,double speedRadS,double torqueNm,double currentA) { }
    record Joint(String name,String body,String unit,double position,double velocity,double effort,boolean effortIsLimit,boolean effortIsEstimate,Double lower,Double upper,Vec pivot,Vec axis) { }
    record Snapshot(double timestepS,List<Contact> contacts,List<Wheel> wheels,List<Motor> motors,List<Joint> joints,List<String> notes,Vec driveOrigin,Vec driveForce,double driveTorqueNm,Vec forward,double speedMps,double yawRateRadS,boolean tireModel,boolean truncated) {
        Snapshot {contacts=List.copyOf(contacts);wheels=List.copyOf(wheels);motors=List.copyOf(motors);joints=List.copyOf(joints);notes=List.copyOf(notes);}
    }
    private final PhysicsWorld world;
    private final ImportedRobotScene scene;
    private final ArticulatedRobot robot;
    private final HardwareMap hardware;
    private final Set<String> drives;
    private final Map<Long,String> bodies=new LinkedHashMap<>();
    private final List<simcore.RobotUrdf.Joint> wheelJoints;
    PhysicsDiagnostics(PhysicsWorld world,ImportedRobotScene scene,ArticulatedRobot robot,HardwareMap hardware,Set<String> drives) {
        this.world=world;this.scene=scene;this.robot=robot;this.hardware=hardware;this.drives=Set.copyOf(drives);
        if(scene!=null&&robot==null)throw new IllegalArgumentException("Imported diagnostics require the articulated robot");
        if(robot!=null)bodies.putAll(robot.diagnosticBodies());else bodies.put(world.chassisBody().nativeId(),"chassis");
        if(world.flexibleIntake()!=null){int i=0;for(var flap:world.flexibleIntake().flaps)for(var segment:flap.segments)bodies.put(segment.body().nativeId(),"flexible-segment-"+i++);}
        wheelJoints=scene==null?List.of():scene.urdf.joints.values().stream().filter(scene::isDriveWheel).sorted(Comparator.comparing(simcore.RobotUrdf.Joint::name)).toList();
    }
    private String name(PhysicsCollisionObject b) {
        if(bodies.containsKey(b.nativeId()))return bodies.get(b.nativeId());
        if(b instanceof RigidBodyControl c&&c.getSpatial()!=null)return c.getSpatial().getName();
        return b instanceof PhysicsRigidBody r&&r.isStatic()?"static surface/obstacle":"dynamic body";
    }
    Snapshot capture() {
        double dt=world.driveImpulse().dt();if(dt<=0)dt=world.space().getAccuracy();
        var contacts=new ArrayList<Contact>();boolean truncated=false;long[] manifolds=world.space().listManifoldIds();int scanned=0;
        outer:for(long m:manifolds) {
            if(scanned++>=MAX_MANIFOLDS){truncated=true;break;}
            long aId=PersistentManifolds.getBodyAId(m),bId=PersistentManifolds.getBodyBId(m);boolean aRobot=bodies.containsKey(aId),bRobot=bodies.containsKey(bId);if(!aRobot&&!bRobot)continue;
            var a=PhysicsCollisionObject.findInstance(aId);var b=PhysicsCollisionObject.findInstance(bId);
            for(long p:PersistentManifolds.listPointIds(m)) {
                if(contacts.size()>=MAX_CONTACTS){truncated=true;break outer;}
                var normal=new Vector3f();ManifoldPoints.getNormalWorldOnB(p,normal);var position=new Vector3f();
                if(aRobot)ManifoldPoints.getPositionWorldOnA(p,position);else {ManifoldPoints.getPositionWorldOnB(p,position);normal.negateLocal();}
                // Last solved impulses/positions, including separation; show the gap rather than fabricate support.
                double load=Math.max(0,ManifoldPoints.getAppliedImpulse(p))/dt;var other=aRobot?b:a;
                boolean supported=other instanceof PhysicsRigidBody ground&&ground.isStatic()&&normal.y>.5&&ManifoldPoints.getDistance1(p)<=.003;
                contacts.add(new Contact(name(aRobot?a:b),name(other),Vec.of(position),Vec.of(normal),load,ManifoldPoints.getDistance1(p),supported,load*Math.max(0,Math.min(10,a.getFriction()*b.getFriction()))));
            }
        }
        var support=world.driveContacts()==null?null:world.driveContacts().snapshot();
        var tireStates=world.tireDrive()==null?List.<TireDrive.State>of():world.tireDrive().states();var wheels=new ArrayList<Wheel>();
        for(var j:wheelJoints.stream().limit(MAX_ITEMS).toList()) {
            var s=support==null?null:support.wheels().stream().filter(w->w.joint().equals(j.name())).findFirst().orElse(null);
            var tire=tireStates.stream().filter(w->w.joint().equals(j.name())).findFirst().orElse(null);
            Vector3f hub=robot.linkFrameWorld(j.child()).getTranslation();
            double load=tire!=null?tire.normalN():s==null?0:s.normalImpulse()/dt,grip=tire!=null?tire.gripN():s==null?0:s.frictionImpulse()/dt;
            boolean nativeWheel=scene.rotatingWheels!=null,supported=tire!=null?tire.supported():s!=null;
            if(nativeWheel){String owner=scene.owners.get(j.child());load=contacts.stream().filter(c->c.body().equals(owner)&&c.staticSupport()).mapToDouble(Contact::loadN).sum();grip=contacts.stream().filter(c->c.body().equals(owner)&&c.staticSupport()).mapToDouble(Contact::gripN).sum();supported=contacts.stream().anyMatch(c->c.body().equals(owner)&&c.staticSupport());}
            wheels.add(new Wheel(j.name(),Vec.of(hub),supported,load,grip,tire==null?null:tire.slipMps(),tire==null?null:tire.forceN(),tire==null?null:tire.lateralN(),tire!=null&&tire.sliding()));
        }
        var allMotors=hardware.getAll(SimDcMotorEx.class).stream().sorted(Comparator.comparing(SimDcMotorEx::getDeviceName)).toList();
        var motors=allMotors.stream().limit(MAX_ITEMS).map(m->new Motor(m.getDeviceName(),m.signedCommandedPower(),m.getOmegaRadS(),m.externalShaftTorque(),m.getCurrent(CurrentUnit.AMPS))).toList();
        var joints=robot==null?List.<Joint>of():robot.diagnosticJoints().stream().map(j->new Joint(j.name(),j.body(),j.type().equals("prismatic")?"m":"rad",j.position(),j.velocity(),j.effort(),j.effortIsLimit(),j.effortIsEstimate(),j.lower(),j.upper(),Vec.of(j.pivot()),Vec.of(j.axis()))).toList();
        truncated|=wheelJoints.size()>MAX_ITEMS||allMotors.size()>MAX_ITEMS||robot!=null&&robot.diagnosticJointCount()>MAX_ITEMS;
        var notes=new ArrayList<String>();var missing=drives.stream().filter(n->hardware.tryGet(SimDcMotorEx.class,n)==null).sorted().toList();
        if(!missing.isEmpty())notes.add("Missing drive motor bindings: "+String.join(", ",missing)+".");
        if(scene!=null&&scene.rotatingWheels!=null)notes.add(wheels.stream().anyMatch(Wheel::supported)?"Rotating native wheels: Bullet owns traction and load transfer.":"Rotating wheels have no sampled static support.");
        else if(support==null)notes.add("Native wheel support unavailable (legacy proxy mode).");
        else if(support.wheels().isEmpty())notes.add(support.scrapingContacts()>0?"Chassis scrapes the floor; drive wheels have no support. Check clearance.":"No native wheel support. Check wheel geometry, axes and floor height.");
        else {
            double load=support.normalImpulse()/dt,grip=support.frictionImpulse()/dt;
            if(load>.01&&grip<.001)notes.add("Wheel support present; available material grip is zero.");
            if(support.scrapingContacts()>0)notes.add("Chassis also scrapes the floor. Inspect belly clearance.");
            if(wheels.stream().anyMatch(Wheel::sliding))notes.add("Tires reached a grip limit; slip may reduce commanded motion.");
        }
        boolean command=drives.stream().map(n->hardware.tryGet(SimDcMotorEx.class,n)).filter(Objects::nonNull).anyMatch(m->Math.abs(m.signedCommandedPower())>.01);
        var velocity=world.chassisBody().isDynamic()?world.chassisBody().getLinearVelocity():Vector3f.ZERO;
        var angular=world.chassisBody().isDynamic()?world.chassisBody().getAngularVelocity():Vector3f.ZERO;
        boolean slow=Math.hypot(velocity.x,velocity.z)<.01&&Math.abs(angular.y)<.03;
        boolean obstacle=contacts.stream().anyMatch(c->!bodies.containsValue(c.other())&&c.normal().y()<.7&&c.loadN()>.01&&c.gapM()<=.003);
        if(command&&slow&&obstacle)notes.add("Drive effort with little motion and obstacle contact: possible blocking.");
        for(var j:joints) {
            double tolerance=j.unit().equals("m")?.002:.015;
            if(j.lower()!=null&&(j.position()<=j.lower()+tolerance||j.position()>=j.upper()-tolerance))notes.add(j.name()+": near a configured travel limit.");
            else if(Math.abs(j.effort())>.01&&Math.abs(j.velocity())<(j.unit().equals("m")?.002:.02)&&contacts.stream().anyMatch(c->c.body().equals(j.body())||c.other().equals(j.body())))notes.add(j.name()+": effort with little motion and contact; possible blocking/load.");
        }
        if(notes.isEmpty())notes.add("Supported robot; no configured diagnostic condition detected.");
        if(world.tireDrive()==null)notes.add(scene!=null&&scene.rotatingWheels!=null?"Brush slip/force unavailable; native wheel contacts own traction.":"Individual tire slip/force unavailable (aggregate drive).");
        if(truncated)notes.add("Diagnostic sampling limit reached; some items/contacts are omitted.");
        var impulse=world.driveImpulse();double step=impulse.dt();var force=step>0?new Vector3f((float)(impulse.xNs()/step),0,(float)(impulse.zNs()/step)):Vector3f.ZERO;
        return new Snapshot(dt,contacts,wheels,motors,joints,notes,Vec.of(world.getChassisPosition()),Vec.of(force),step>0?impulse.yawNms()/step:0,Vec.of(world.getChassisRotation().mult(Vector3f.UNIT_X)),Math.hypot(velocity.x,velocity.z),angular.y,world.tireDrive()!=null,truncated);
    }
    private static Map<String,Object> row(Object... pairs){var out=new LinkedHashMap<String,Object>();for(int i=0;i<pairs.length;i+=2)out.put((String)pairs[i],pairs[i+1]);return out;}
    private static List<Double> vec(Vec v){return List.of(v.x(),v.y(),v.z());}
    static Map<String,Object> data(Snapshot s) {
        return row("schema_version",1,"engine","Minie 9.0.3 / Bullet","scope","Last physics tick; modeled motor effort/current. Read-only diagnostics, not measured hardware accuracy.","units",row("position","m","velocity","m/s or rad/s","force","N","torque","N*m","current","A","timestep","s"),"timestep_s",s.timestepS(),"truncated",s.truncated(),"notes",s.notes(),"tire_model",s.tireModel(),"drive",row("origin_m",vec(s.driveOrigin()),"force_n",vec(s.driveForce()),"yaw_torque_nm",s.driveTorqueNm(),"speed_mps",s.speedMps(),"yaw_rate_rad_s",s.yawRateRadS()),
            "contacts",s.contacts().stream().map(c->row("body",c.body(),"other",c.other(),"position_m",vec(c.position()),"normal",vec(c.normal()),"normal_load_n",c.loadN(),"gap_m",c.gapM())).toList(),
            "wheels",s.wheels().stream().map(w->row("joint",w.joint(),"hub_m",vec(w.hub()),"supported",w.supported(),"normal_load_n",w.loadN(),"grip_limit_n",w.gripN(),"slip_mps",w.slipMps(),"longitudinal_force_n",w.forceN(),"lateral_force_n",w.lateralN(),"sliding",w.sliding())).toList(),
            "motors",s.motors().stream().map(m->row("name",m.name(),"signed_command",m.command(),"shaft_speed_rad_s",m.speedRadS(),"modeled_shaft_torque_nm",m.torqueNm(),"modeled_current_a",m.currentA())).toList(),
            "joints",s.joints().stream().map(j->row("name",j.name(),"body",j.body(),"position_unit",j.unit(),"position",j.position(),"velocity",j.velocity(),"effort",j.effort(),"effort_kind",j.effortIsEstimate()?"modeled spring effort":j.effortIsLimit()?"native motor effort limit":"applied impulse effort","effort_unit",j.unit().equals("m")?"N":"N*m","lower",j.lower(),"upper",j.upper(),"pivot_m",vec(j.pivot()),"axis",vec(j.axis()))).toList());
    }
}
