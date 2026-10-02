package simrunner;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import simcore.RobotUrdf;
import simcore.SimDcMotorEx;
import java.util.*;

/** Capabilities come from configured bindings and joint types, never visual mesh names. */
final class RobotMotionPlan {
    record Movement(String label, double forward, double left, double turn) { }
    record Mechanism(RobotUrdf.Joint joint, List<RobotUrdf.Actuator> actuators, boolean servo) { }
    record Item(String id,String group,String label,String part) { }
    static String driveId(Movement m){return "drive/"+m.label().toLowerCase(Locale.ROOT).replace(' ','-');}
    static String jointGroup(Mechanism m){return "joint/"+m.joint().name();}
    List<Item> items(){
        var out=new ArrayList<Item>();
        for(var d:drive)out.add(new Item(driveId(d),driveId(d),d.label(),""));
        for(var m:mechanisms){String group=jointGroup(m);out.add(new Item(group+"/first",group,m.joint().name()+" · first target",m.joint().child()));out.add(new Item(group+"/second",group,m.joint().name()+" · return target",m.joint().child()));}
        return List.copyOf(out);
    }
    final List<Movement> drive;
    final List<Mechanism> mechanisms;
    final List<String> notes;
    final Set<String> driveNames;
    final String drivetrain;

    RobotMotionPlan(RobotUrdf urdf, SimConfig config, HardwareMap hardware) {
        drivetrain=config.drive==null?"Mecanum":"Differential / tank";
        driveNames=Set.copyOf(config.drive==null?DriveGeometry.MOTORS:config.drive.motorNames());
        var notes=new ArrayList<String>();
        notes.add(config.driveContacts!=null&&config.driveContacts.enabled()
            ?"Native wheel support: drive needs wheel collision shapes on a static floor. Belly scraping and wall friction remain active."
            :"Legacy drive support: wheel collision geometry is omitted. Choose Wheel support model in Physics assumptions to review native wheel contacts.");
        var wheels=new HashSet<String>();var bound=new HashSet<String>();
        for(var tx:urdf.transmissions.values()) {
            var j=urdf.joints.get(tx.joint());
            for(var a:tx.actuators())if(driveNames.contains(a.name())&&j.type().equals("continuous")) {
                bound.add(a.name());wheels.add(j.child());
            }
        }
        boolean changed;
        do {changed=false;for(var j:urdf.joints.values())if(wheels.contains(j.parent()))changed|=wheels.add(j.child());}while(changed);
        var missing=new TreeSet<>(driveNames);missing.removeAll(bound);
        for(String name:driveNames)if(hardware.tryGet(SimDcMotorEx.class,name)==null)missing.add(name);
        var drive=new ArrayList<Movement>();
        if(missing.isEmpty()) {
            drive.add(new Movement("Forward",1,0,0));drive.add(new Movement("Backward",-1,0,0));
            if(config.drive==null){drive.add(new Movement("Strafe left",0,1,0));drive.add(new Movement("Strafe right",0,-1,0));}
            drive.add(new Movement("Turn left",0,0,1));drive.add(new Movement("Turn right",0,0,-1));
        } else notes.add("Drive demo unavailable: bind continuous drive wheels to "+String.join(", ",missing)+" in Parts and movement. Select the drive type explicitly; mesh shape does not establish wheel capability.");
        var mechanisms=new ArrayList<Mechanism>();
        for(var j:urdf.joints.values().stream().sorted(Comparator.comparing(RobotUrdf.Joint::name)).toList()) {
            if(j.type().equals("fixed"))continue;
            if(wheels.contains(j.child()))continue;
            if(j.mimic()!=null){notes.add(j.name()+": passive follower of "+j.mimic()+"; demonstrated with its powered source.");continue;}
            var actuators=urdf.transmissions.values().stream().filter(t->t.joint().equals(j.name())).flatMap(t->t.actuators().stream()).toList();
            if(actuators.isEmpty()){notes.add(j.name()+": unbound/passive; bind a motor or servo if this should be powered.");continue;}
            if(actuators.stream().anyMatch(a->driveNames.contains(a.name())))throw new IllegalArgumentException("Drive motor also drives a non-wheel joint: "+j.name());
            boolean servo=actuators.stream().anyMatch(a->hardware.tryGet(Servo.class,a.name())!=null);
            if(servo&&(actuators.size()!=1||!config.servoPhysics.containsKey(actuators.get(0).name())))throw new IllegalArgumentException(j.name()+": configure one servo and its torque, speed and travel in Physics assumptions.");
            if(servo&&j.type().equals("prismatic"))throw new IllegalArgumentException(j.name()+": servoPhysics describes rotational travel; use a supported motor-driven slide.");
            for(var a:actuators)if(hardware.tryGet(SimDcMotorEx.class,a.name())==null&&hardware.tryGet(Servo.class,a.name())==null)throw new IllegalArgumentException("Missing motor/servo "+a.name()+" for "+j.name()+". Check the selected project or its binding.");
            mechanisms.add(new Mechanism(j,actuators,servo));
        }
        var used=new HashSet<String>();
        for(var m:mechanisms)for(var a:m.actuators())if(!used.add(a.name()))throw new IllegalArgumentException("Actuator drives multiple independent mechanisms: "+a.name());
        this.drive=List.copyOf(drive);this.mechanisms=List.copyOf(mechanisms);this.notes=List.copyOf(notes);
    }
    boolean empty(){return drive.isEmpty()&&mechanisms.isEmpty();}
    String summary(){return (drive.isEmpty()?"Drivetrain not ready":drivetrain+": "+String.join(", ",drive.stream().map(Movement::label).toList()))
        +"\n"+mechanisms.size()+" powered mechanisms: "+String.join(", ",mechanisms.stream().map(m->m.joint().name()).toList())
        +(notes.isEmpty()?"":"\n"+String.join("\n",notes));}
}
