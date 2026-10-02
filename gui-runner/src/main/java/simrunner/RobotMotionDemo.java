package simrunner;

import com.jme3.asset.AssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.*;
import com.jme3.scene.Node;
import com.qualcomm.robotcore.hardware.*;
import physics.*;
import simcore.*;
import java.nio.file.*;
import java.util.*;

/** Bounded scripted inputs applied to the production motors and native robot, without TeamCode. */
final class RobotMotionDemo implements AutoCloseable {
    static final float DT=1f/480;
    record Setup(ModelProfile profile, RobotUrdf urdf, SimConfig config, HardwareMap hardware,
                 RobotMotionPlan plan, String hardwareLabel, String context) { }
    record Action(String id,String group,String label, RobotMotionPlan.Movement drive, RobotMotionPlan.Mechanism mechanism, double target) { }
    final String runId=UUID.randomUUID().toString();
    final String selection;
    final Setup setup;
    final PhysicsSpace space;
    final Node root=new Node("motion-demo");
    final PhysicsWorld world;
    final ImportedRobotScene scene;
    final ArticulatedRobot robot;
    final List<Action> actions=new ArrayList<>();
    final List<Map<String,Object>> observations=new ArrayList<>();
    private final MecanumKinematics kinematics;
    private final List<Double> wheelRadii;
    private int index=-1;
    private double clock, elapsed;
    private Vector3f startPosition;
    private Quaternion startRotation;
    private double startJoint, peakJoint, peakTravel;
    private boolean finished, paused, stopped;
    String failure="";

    static Setup setup(Path file,Path project)throws Exception {
        var profile=new ModelProfile(file,false);
        if(!profile.kind.equals("robot"))throw new IllegalArgumentException("Choose a robot model for the motion demo.");
        if(!FieldPackage.maps(FieldPackage.map(profile.data.get("migration")).get("pending")).isEmpty())throw new IllegalArgumentException("Resolve CAD migration choices in Parts and movement before the motion demo.");
        Path urdfFile=profile.artifact("robot");var urdf=RobotUrdf.parse(urdfFile);
        var runtime=new LinkedHashMap<>(profile.runtime);runtime.put("urdf",urdfFile.toString());var config=SimConfig.parse(profile.directory,runtime);
        if(config.totalMassKg!=null)urdf=urdf.withTotalMassKg(config.totalMassKg);
        HardwareMap hardware;String label;Object context;
        if(project!=null) {
            var settings=MiniJson.parseObject(Files.readString(project.resolve("sim.config")));
            if(!(settings.get("robotConfig") instanceof String xml)||!(settings.get("presetMotors") instanceof String preset))throw new IllegalArgumentException("Choose a project with robotConfig and presetMotors paths in sim.config.");
            Path xmlFile=project.resolve(xml),presetFile=project.resolve(preset);
            var xmlConfig=RobotConfigXml.parse(xmlFile.toFile());var motorPreset=PresetRobotConfig.load(presetFile);
            hardware=HardwareMapBuilder.build(xmlConfig,motorPreset);
            var fallback=xmlConfig.devices.stream().filter(d->RobotConfigXml.resolveType(d.tag)==RobotConfigXml.DeviceType.MOTOR&&!motorPreset.motors.containsKey(d.name)).map(d->d.name).toList();
            label=fallback.isEmpty()?"Selected project hardware and motor presets":"Selected project hardware; generic fallback motor specs for "+String.join(", ",fallback)+". Configure motor presets to verify real effort.";
            context=List.of(project.toAbsolutePath().normalize().toString(),settings,ModelProfile.hash(xmlFile),ModelProfile.hash(presetFile));
        } else {
            hardware=new HardwareMap();
            for(var tx:urdf.transmissions.values())for(var a:tx.actuators()) {
                if(config.servoPhysics.containsKey(a.name()))hardware.register(a.name(),new SimServo(a.name()));
                else hardware.register(a.name(),new SimDcMotorEx(a.name(),new MotorSpec("generic demo",1,2,9.2,30,12,500)));
            }
            HardwareMapBuilder.getBatteryModel().vInternal=12.6;HardwareMapBuilder.getBatteryModel().rBattery=.15;
            label="Generic demo motors; saved servo settings. Choose a project to verify device types and motor presets.";
            context="generic-demo-motors-v1";
        }
        urdf.validateHardwareMap(hardware);
        var plan=new RobotMotionPlan(urdf,config,hardware);
        return new Setup(profile,urdf,config,hardware,plan,label,GuidedSetupSession.hash(List.of("motion-demo-v2",profile.digest,context)));
    }

    RobotMotionDemo(Setup setup,AssetManager assets)throws Exception {
        this(setup,assets,"");
    }
    RobotMotionDemo(Setup setup,AssetManager assets,String selection)throws Exception {
        this.selection=selection;
        if(!selection.isEmpty()&&setup.plan.items().stream().noneMatch(i->i.group().equals(selection)))throw new IllegalArgumentException("This movement is no longer configured. Return to the guide and choose a current movement.");
        this.setup=setup;space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);space.setAccuracy(DT);
        world=new PhysicsWorld(assets,root,space);
        try {
            scene=new ImportedRobotScene(setup.urdf,setup.profile.artifact("robot"),setup.hardware,assets,setup.config.vhacdMaxHulls,setup.plan.driveNames);
            setup.profile.configure(scene);scene.tireContacts=setup.config.tires!=null;scene.flexibleIntake=setup.config.flexibleIntake;
            scene.driveContacts=setup.config.driveContacts;
            CollisionAudit.inspect(scene).requireUsable();
            robot=new ArticulatedRobot(scene,world,root,new Vector3f(0,(float)setup.config.startHeightM,0),setup.config.servoPhysics);
            // Lift the whole assembly together to the neutral floor, retaining all joint frames.
            double bottom=Double.POSITIVE_INFINITY;
            for(var b:space.getRigidBodyList()) {
                var shape=com.jme3.bullet.util.DebugShapeFactory.getDebugShape(b.getCollisionShape());shape.setLocalRotation(b.getPhysicsRotation());shape.setLocalTranslation(b.getPhysicsLocation());shape.updateGeometricState();
                if(shape.getWorldBound() instanceof com.jme3.bounding.BoundingBox box)bottom=Math.min(bottom,box.getCenter().y-box.getYExtent());
            }
            if(bottom<.003)for(var b:space.getRigidBodyList())b.setPhysicsLocation(b.getPhysicsLocation().add(0,(float)(.003-bottom),0));
            var floor=new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(20,.02f,20)),0);floor.setPhysicsLocation(new Vector3f(0,-.02f,0));floor.setFriction(.6f);space.add(floor);
            if(setup.config.tires!=null)world.installTires(new TireDrive(world,setup.hardware,scene,setup.config.drive,setup.config.tires));
            if(setup.config.flexibleIntake!=null)world.installFlexibleIntake(new FlexibleIntake(world,scene,robot,setup.config.flexibleIntake));
            if(setup.config.calibration!=null){var calibration=CalibrationProfile.load(setup.profile.directory.resolve(setup.config.calibration));calibration.applyHardware(setup.hardware);calibration.applyDrive(world);}
            var dimensions=DriveGeometry.resolve(setup.urdf,setup.config.driveGeometry);wheelRadii=dimensions.wheelRadii();kinematics=new MecanumKinematics(dimensions.trackWidthM(),dimensions.wheelbaseM(),2.);
            for(var m:setup.hardware.getAll(SimDcMotorEx.class))m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Initial servo target follows the joint's current zero pose where reachable.
            for(var m:setup.plan.mechanisms)if(m.servo())servo(m,0);
            for(var movement:setup.plan.drive){String id=RobotMotionPlan.driveId(movement);actions.add(new Action(id,id,movement.label(),movement,null,0));}
            for(var m:setup.plan.mechanisms) {
                double low,high;
                if(m.servo()) {
                    var a=m.actuators().get(0);double origin=m.joint().lower()==null?0:m.joint().lower();double end=origin+setup.config.servoPhysics.get(a.name()).travelRad()/a.mechanicalReduction();
                    low=Math.min(origin,end);high=Math.max(origin,end);
                } else {double span=m.joint().type().equals("prismatic")?.04:.6;double center=m.joint().lower()==null?0:Math.max(m.joint().lower(),Math.min(m.joint().upper(),0));low=center-span;high=center+span;}
                if(m.joint().lower()!=null){low=Math.max(low,m.joint().lower());high=Math.min(high,m.joint().upper());}
                if(high-low<1e-6){observations.add(Map.of("action",m.joint().name(),"outcome","needs setup","detail","No reachable travel; check joint limits, servo travel and signed gearing."));continue;}
                if(m.servo()){double span=high-low;low+=span*.2;high-=span*.2;}
                String unit=m.joint().type().equals("prismatic")?"m":"rad";
                String group=RobotMotionPlan.jointGroup(m);
                actions.add(new Action(group+"/first",group,String.format(Locale.ROOT,"%s — toward %.3f %s",m.joint().name(),high,unit),null,m,high));
                actions.add(new Action(group+"/second",group,String.format(Locale.ROOT,"%s — toward %.3f %s",m.joint().name(),low,unit),null,m,low));
            }
            if(!selection.isEmpty()){actions.removeIf(a->!a.group.equals(selection));observations.clear();if(actions.isEmpty())throw new IllegalArgumentException("No reachable movement for this selection. Check joint limits and servo travel.");}
            world.driveControllerEnabled=!setup.plan.drive.isEmpty();
            if(actions.isEmpty())finished=true;
        }catch(Exception e){space.destroy();throw e;}
    }
    Action action(){return index<0||index>=actions.size()?null:actions.get(index);}
    boolean finished(){return finished;}
    boolean paused(){return paused;}
    void pause(){if(!finished){paused=!paused;stopMotors();}}
    void stop(){stopMotors();if(finished)return;paused=true;stopped=true;finished=true;}
    long completedMovements(){return observations.stream().filter(row->row.containsKey("forward_m")||row.containsKey("start")).count();}
    void stopMotors(){for(var m:setup.hardware.getAll(SimDcMotorEx.class))m.setPower(0);world.driveChassis(new MecanumKinematics.ChassisVelocity(0,0,0),DT);}
    private void servo(RobotMotionPlan.Mechanism mechanism,double target) {
        var a=mechanism.actuators().get(0);double low=mechanism.joint().lower()==null?0:mechanism.joint().lower();
        setup.hardware.get(Servo.class,a.name()).setPosition((target-low)*a.mechanicalReduction()/setup.config.servoPhysics.get(a.name()).travelRad());
    }
    private void begin(){index++;elapsed=0;stopMotors();if(index>=actions.size()){finished=true;return;}startPosition=world.getChassisPosition();startRotation=world.getChassisRotation();peakTravel=0;startJoint=action().mechanism==null?0:robot.jointPosition(action().mechanism.joint().name());peakJoint=0;}
    void tick() {
        if(paused||finished)return;
        try {
            clock+=DT;
            if(index<0&&clock>=.6)begin();
            if(finished)return;
            var a=action();elapsed+=DT;stopMotors();
            boolean powering=a!=null&&elapsed<.85;
            if(powering&&a.drive!=null&&peakTravel<.25) {
                if(setup.config.drive!=null){var d=setup.config.drive;setup.hardware.get(SimDcMotorEx.class,d.leftMotor()).setPower(.18*(a.drive.forward()-a.drive.turn())*d.leftShaftSign());setup.hardware.get(SimDcMotorEx.class,d.rightMotor()).setPower(.18*(a.drive.forward()+a.drive.turn())*d.rightShaftSign());}
                else {double[] power=kinematics.inverse(a.drive.forward()*.36,a.drive.left()*.36,a.drive.turn()*.7);for(int i=0;i<4;i++)setup.hardware.get(SimDcMotorEx.class,DriveGeometry.MOTORS.get(i)).setPower(power[i]);}
            } else if(a!=null&&a.mechanism!=null) {
                var mechanism=a.mechanism;double q=robot.jointPosition(mechanism.joint().name());
                if(mechanism.servo())servo(mechanism,a.target);
                else {
                    double error=a.target-q;double span=mechanism.joint().type().equals("prismatic")?.04:.6;
                    for(var actuator:mechanism.actuators()) {
                        var motor=setup.hardware.get(SimDcMotorEx.class,actuator.name());double rate=motor.getOmegaRadS()/actuator.mechanicalReduction();
                        double demand=(error-.15*rate)/span;motor.setPower(.15*Math.max(-1,Math.min(1,demand))*Math.signum(actuator.mechanicalReduction()));
                    }
                }
            }
            var battery=HardwareMapBuilder.getBatteryModel();var motors=setup.hardware.getAll(SimDcMotorEx.class);double voltage=battery.solveBatteryVoltage(motors.stream().map(m->new BatteryModel.MotorState(m.signedCommandedPower(),m.getSpec(),m.getOmegaRadS())).toList());
            for(var motor:motors)motor.integrate(voltage,DT,Math.round(clock*1000));
            if(!setup.plan.drive.isEmpty()) {
                MecanumKinematics.ChassisVelocity velocity;
                if(setup.config.drive!=null)velocity=setup.config.drive.velocity(setup.hardware);
                else {double[] v=new double[4];for(int i=0;i<4;i++)v[i]=setup.hardware.get(SimDcMotorEx.class,DriveGeometry.MOTORS.get(i)).getOmegaRadS()*wheelRadii.get(i);velocity=kinematics.forwardFromWheelSpeeds(v[0],v[1],v[2],v[3]);}
                world.driveChassis(velocity,DT);
            }
            space.update(DT,0);scene.update();world.updateFlexibleVisuals(DT);
            for(var b:space.getRigidBodyList())if(!Vector3f.isValidVector(b.getPhysicsLocation())||b.getPhysicsLocation().length()>30||b.isDynamic()&&(!Vector3f.isValidVector(b.getLinearVelocity())||b.getLinearVelocity().length()>50||!Vector3f.isValidVector(b.getAngularVelocity())))throw new IllegalArgumentException("Unstable or out-of-range native movement; revise collisions, joints and mass.");
            if(a!=null){peakTravel=Math.max(peakTravel,world.getChassisPosition().distance(startPosition));if(a.mechanism!=null)peakJoint=Math.max(peakJoint,Math.abs(robot.jointPosition(a.mechanism.joint().name())-startJoint));}
            if(a!=null&&elapsed>=1.15){observe(a);begin();}
        }catch(Exception e){failure=e.getMessage()==null?e.toString():e.getMessage();observations.add(Map.of("action",action()==null?"Settle":action().label,"outcome","needs attention","detail",failure));stop();}
    }
    private void observe(Action a) {
        var row=new LinkedHashMap<String,Object>();row.put("id",a.id);row.put("group",a.group);row.put("action",a.label);
        boolean moved,correct;
        if(a.drive!=null) {
            Vector3f local=startRotation.inverse().mult(world.getChassisPosition().subtract(startPosition));Vector3f forward=startRotation.inverse().mult(world.getChassisRotation().mult(Vector3f.UNIT_X));double yaw=Math.atan2(-forward.z,forward.x);
            double value=a.drive.turn()!=0?yaw:a.drive.left()!=0?-local.z:local.x;
            double sign=a.drive.turn()!=0?a.drive.turn():a.drive.left()!=0?a.drive.left():a.drive.forward();
            moved=Math.abs(value)>(a.drive.turn()!=0?.02:.005);correct=value*sign>0;
            row.put("forward_m",(double)local.x);row.put("left_m",(double)-local.z);row.put("yaw_rad",yaw);
            if(world.driveContacts()!=null){var support=world.driveContacts().snapshot();row.put("supported_wheels",support.wheels().size());row.put("scraping_contacts",support.scrapingContacts());row.put("support_diagnosis",support.diagnosis());}
        } else {
            double q=robot.jointPosition(a.mechanism.joint().name());double value=q-startJoint;
            moved=peakJoint>(a.mechanism.joint().type().equals("prismatic")?.002:.01);correct=(a.target-startJoint)*value>=0;
            row.put("start",startJoint);row.put("end",q);row.put("target",a.target);row.put("peak_travel",peakJoint);row.put("unit",a.mechanism.joint().type().equals("prismatic")?"m":"rad");
        }
        row.put("outcome",!moved?"no clear movement":correct?"movement observed":"unexpected direction");row.put("detail",!moved?row.getOrDefault("support_diagnosis","Inspect bindings, limits, gearing, contact and motor effort."):correct?"Compare this movement with your expected physical robot.":"Check axes, signs, gearing and bindings.");observations.add(row);
    }
    Map<String,Object> report(){var report=new LinkedHashMap<String,Object>();report.put("schema_version",2);report.put("run_id",runId);report.put("selection",selection);report.put("model_digest",setup.profile.digest);report.put("context",setup.context);report.put("hardware",setup.hardwareLabel);report.put("complete",finished&&!stopped);report.put("status",failure.isEmpty()?stopped?"stopped":finished?"finished":"running":"needs attention");report.put("observations",List.copyOf(observations));report.put("notes",setup.plan.notes);report.put("scope","Scripted native motion observations; not collision review or measured physical accuracy.");return report;}
    String status(){return failure.isEmpty()?finished?stopped?"Stopped":"Demo complete":paused?"Paused":action()==null?"Settling on the demo floor":action().label:failure;}
    public void close(){stopMotors();space.destroy();}
}
