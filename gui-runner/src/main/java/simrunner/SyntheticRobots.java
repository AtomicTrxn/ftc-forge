package simrunner;

import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.zip.*;

/** Small, inspectable CAD packages. Never presented as measured hardware or reviewed user models. */
final class SyntheticRobots {
    static final String VERSION="synthetic-robots-v1";
    record Dimensions(double massKg,double radiusM,double trackM,double mechanismReduction) {
        static Dimensions standard(){return new Dimensions(3,.045,.26,1);}
        Dimensions {
            for(double v:new double[]{massKg,radiusM,trackM,Math.abs(mechanismReduction)})
                if(!Double.isFinite(v)||v<=0)throw new IllegalArgumentException("Synthetic dimensions must be finite and positive");
            if(radiusM<=.031||trackM<=.22)throw new IllegalArgumentException("Synthetic wheels must clear the chassis");
        }
    }
    static String urdf(boolean differential,boolean mechanisms,boolean intake,Dimensions d) {
        var xml=new StringBuilder("<robot name='Synthetic validation robot'>");
        xml.append(box("base",".3 .2 .06",d.massKg()*2/3));
        var names=differential?List.of("leftDrive","rightDrive"):DriveGeometry.MOTORS;
        for(int i=0;i<names.size();i++) {
            double mass=d.massKg()/3/names.size(),r=d.radiusM(),width=.02;
            xml.append("<link name='wheel").append(i).append("'><inertial><mass value='").append(mass)
                .append("'/><inertia ixx='").append(mass*(3*r*r+width*width)/12).append("' iyy='").append(mass*r*r/2)
                .append("' izz='").append(mass*(3*r*r+width*width)/12).append("' ixy='0' ixz='0' iyz='0'/></inertial>");
            String geometry="<origin rpy='1.5707963267948966 0 0'/><geometry><cylinder radius='"+r+"' length='.02'/></geometry>";
            xml.append("<visual>").append(geometry).append("</visual><collision>").append(geometry).append("</collision></link>");
            xml.append(joint("wheel_joint"+i,"wheel"+i,"continuous",(i<2?.1:-.1)+" "+(i%2==0?d.trackM()/2:-d.trackM()/2)+" 0","0 1 0",null,null));
            xml.append(transmission("wheel_joint"+i,names.get(i),1));
        }
        if(mechanisms) {
            xml.append(box("arm",".06 .03 .03",.05)).append(joint("arm_joint","arm","revolute","0 0 .3","0 0 1",-.3,.3)).append(transmission("arm_joint","armMotor",d.mechanismReduction()));
            xml.append(box("slide",".06 .03 .03",.05)).append(joint("slide_joint","slide","prismatic",".3 0 .3","1 0 0",0.,.08)).append(transmission("slide_joint","slideMotor",d.mechanismReduction()));
            xml.append(box("gate",".06 .03 .03",.05)).append(joint("gate_joint","gate","revolute","-.3 0 .3","0 0 1",0.,.5)).append(transmission("gate_joint","gateServo",1));
        }
        if(intake)xml.append(box("intake",".06 .03 .03",.05)).append(joint("intake_joint","intake","continuous","0 0 .4","0 0 1",null,null)).append(transmission("intake_joint","intakeMotor",d.mechanismReduction()));
        return xml.append("</robot>").toString();
    }
    static String box(String name,String size,double mass) {
        var s=Arrays.stream(size.split(" ")).mapToDouble(Double::parseDouble).toArray();
        String geometry="<geometry><box size='"+size+"'/></geometry>";
        return "<link name='"+name+"'><inertial><mass value='"+mass+"'/><inertia ixx='"+mass*(s[1]*s[1]+s[2]*s[2])/12+"' iyy='"+mass*(s[0]*s[0]+s[2]*s[2])/12+"' izz='"+mass*(s[0]*s[0]+s[1]*s[1])/12+"' ixy='0' ixz='0' iyz='0'/></inertial><visual>"+geometry+"</visual><collision>"+geometry+"</collision></link>";
    }
    static String joint(String name,String child,String type,String xyz,String axis,Double low,Double high) {
        return "<joint name='"+name+"' type='"+type+"'><parent link='base'/><child link='"+child+"'/><origin xyz='"+xyz+"'/><axis xyz='"+axis+"'/>"+(low==null?"":"<limit lower='"+low+"' upper='"+high+"' effort='5' velocity='2'/>")+"</joint>";
    }
    static String transmission(String joint,String motor,double reduction){return "<transmission name='"+joint+"_tx'><joint name='"+joint+"'/><actuator name='"+motor+"'><mechanicalReduction>"+reduction+"</mechanicalReduction></actuator></transmission>";}
    static Path zip(Path file,String entry,String content)throws Exception {
        return zip(file,Map.of(entry,content));
    }
    static Path zip(Path file,Map<String,String> entries)throws Exception {
        Files.createDirectories(file.getParent());
        try(var out=new ZipOutputStream(Files.newOutputStream(file))){for(var entry:new TreeMap<>(entries).entrySet()){var e=new ZipEntry(entry.getKey());e.setTime(0);out.putNextEntry(e);out.write(entry.getValue().getBytes(StandardCharsets.UTF_8));out.closeEntry();}}return file;
    }
    static GuidedSetupController importRobot(Path folder,boolean differential,boolean mechanisms,boolean intake,Dimensions d)throws Exception {
        var c=new GuidedSetupController(folder.resolve("library"),null);c.session.start("robot",null);
        c.load("robot","fresh",zip(folder.resolve("robot.zip"),"robot.urdf",urdf(differential,mechanisms,intake,d)),null);
        var p=c.session.profile("robot");var runtime=FieldPackage.map(p.get("runtime"));
        if(differential)runtime.put("drive",Map.of("type","differential","left_motor","leftDrive","right_motor","rightDrive","wheel_radius_m",d.radiusM(),"track_width_m",d.trackM(),"left_shaft_sign",-1,"right_shaft_sign",1));
        if(mechanisms)runtime.put("servoPhysics",Map.of("gateServo",Map.of("stall_torque_nm",1.,"no_load_speed_rad_s",3.,"travel_rad",.5,"position_gain_per_s",8.,"velocity_gain_nm_per_rad_s",.2,"deadband_rad",.001)));
        c.update("robot",p);c.compile("robot");return c;
    }
    static GuidedSetupController rotatingRobot(Path tmp,boolean suspension)throws Exception {
        String xml=SyntheticRobots.urdf(false,false,false,SyntheticRobots.Dimensions.standard())
            .replace("left_front_drive","leftDrive").replace("left_back_drive","leftDrive").replace("right_front_drive","rightDrive").replace("right_back_drive","rightDrive");
        for(int i:new int[]{0,2})xml=xml.replace("<axis xyz='0 1 0'/></joint><transmission name='wheel_joint"+i,"<axis xyz='0 -1 0'/></joint><transmission name='wheel_joint"+i);
        if(suspension)for(int i=0;i<4;i++) {
            String x=i<2?".1":"-.1",y=i%2==0?".13":"-.13";
            String mount="<parent link='base'/><child link='wheel"+i+"'/><origin xyz='"+(i<2?"0.1":"-0.1")+" "+(i%2==0?"0.13":"-0.13")+" 0'/>";
            xml=xml.replace(mount,"<parent link='carrier"+i+"'/><child link='wheel"+i+"'/><origin xyz='0 0 0'/>");
            xml=xml.replace("</robot>",SyntheticRobots.box("carrier"+i,".01 .01 .01",.03)+SyntheticRobots.joint("spring"+i,"carrier"+i,"prismatic",x+" "+y+" 0","0 0 1",-.02,.04)+"</robot>");
        }
        var c=new GuidedSetupController(tmp.resolve(suspension?"springs":"rigid"),null);c.session.start("robot",null);c.load("robot","fresh",SyntheticRobots.zip(tmp.resolve(suspension?"spring.zip":"rigid.zip"),"robot.urdf",xml),null);
        var p=c.session.profile("robot");var runtime=FieldPackage.map(p.get("runtime"));runtime.put("drive",Map.of("type","differential","left_motor","leftDrive","right_motor","rightDrive","track_width_m",.26,"wheel_radius_m",.045,"left_shaft_sign",-1,"right_shaft_sign",1));
        runtime.put("rotating_wheels",Map.of("reflected_motor_inertia_kg_m2",.0015));FieldPackage.map(runtime.get("drive_contacts")).put("enabled",false);FieldPackage.map(p.get("parameters")).put("chassis_lock_level",false);
        if(suspension)for(int i=0;i<4;i++){var s=FieldPackage.map(FieldPackage.map(FieldPackage.map(p.get("entities")).get("carrier"+i)).get("settings"));s.put("joint_spring_n_per_m",500.);s.put("joint_damping_ns_per_m",5.);s.put("joint_rest_m",0.);}
        c.update("robot",p);c.compile("robot");return c;
    }
    /** Deliberately expressed in millimeters to exercise explicit source units. */
    static String stlBox() {
        double[][] v={{0,0,0},{100,0,0},{100,80,0},{0,80,0},{0,0,60},{100,0,60},{100,80,60},{0,80,60}};
        int[][] faces={{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},{1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
        var stl=new StringBuilder("solid fixture\n");for(var face:faces){stl.append("facet normal 0 0 0\nouter loop\n");for(int i:face)stl.append("vertex ").append(v[i][0]).append(' ').append(v[i][1]).append(' ').append(v[i][2]).append('\n');stl.append("endloop\nendfacet\n");}return stl.append("endsolid fixture\n").toString();
    }
}
