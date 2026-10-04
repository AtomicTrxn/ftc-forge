package simrunner;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.*;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.*;
import com.jme3.scene.Node;
import com.jme3.system.NativeLibraryLoader;
import physics.MecanumKinematics;
import java.nio.file.*;
import java.util.*;
import static simrunner.ValidationReport.require;

/** Reusable native scenarios with generated, portable CAD and explicit expected behavior. */
public final class SimulatorValidation {
    static final float DT=1f/120;
    private static final DesktopAssetManager ASSETS=new DesktopAssetManager(true);
    static void complete(RobotMotionDemo demo) {
        for(int i=0;i<30000&&!demo.finished();i++) {
            demo.tick();
            for(var m:demo.setup.plan().mechanisms)if(m.joint().lower()!=null) {
                double q=demo.robot.jointPosition(m.joint().name());
                require(q>=m.joint().lower()-.01&&q<=m.joint().upper()+.01,"Joint limit violation: "+m.joint().name()+" q="+q);
            }
        }
        require(demo.finished()&&demo.failure.isEmpty(),"Incomplete/unstable demo: "+demo.failure);
    }
    static Map<String,Object> movements(RobotMotionDemo.Setup setup,int expected)throws Exception {
        try(var demo=new RobotMotionDemo(setup,ASSETS)) {
            require(demo.actions.size()==expected&&expected>0,"Expected "+expected+" actions, got "+demo.actions.size());complete(demo);
            require(demo.observations.size()==expected,"Missing observations");
            for(var r:demo.observations)require("movement observed".equals(r.get("outcome")),"Incorrect/no movement: "+ProfileIO.json(r));
            return Map.of("observations",demo.observations,"actions",expected,"model_digest",setup.profile().digest,"hardware",setup.hardwareLabel(),"native_mass_kg",demo.space.getRigidBodyList().stream().filter(PhysicsRigidBody::isDynamic).mapToDouble(PhysicsRigidBody::getMass).sum());
        }
    }
    static void advance(RobotMotionDemo demo,double seconds,float dt){for(int i=0;i<Math.round(seconds/dt);i++)demo.space.update(dt,0);}
    static PhysicsRigidBody floor(RobotMotionDemo demo){return demo.space.getRigidBodyList().stream().filter(b->b.isStatic()&&b.getPhysicsLocation().y<0).findFirst().orElseThrow();}
    static Map<String,Object> contactScenario(RobotMotionDemo.Setup setup,String kind)throws Exception {
        try(var demo=new RobotMotionDemo(setup,ASSETS,"drive/forward")) {
            if(kind.equals("airborne")) {
                demo.space.setGravity(Vector3f.ZERO);var b=demo.world.chassisBody();b.setPhysicsLocation(b.getPhysicsLocation().add(0,2,0));
                demo.world.driveChassis(new MecanumKinematics.ChassisVelocity(2,0,1),DT);advance(demo,.5,DT);
                require(b.getLinearVelocity().length()<.001&&demo.world.driveContacts().snapshot().wheels().isEmpty(),"Airborne drive invented traction");
                b.applyCentralImpulse(new Vector3f(b.getMass(),0,0));float start=b.getPhysicsLocation().x;advance(demo,.5,DT);double travel=b.getPhysicsLocation().x-start;
                require(travel>.45,"Airborne controller canceled external push: "+travel);return Map.of("supported_wheels",0,"push_travel_m",travel);
            }
            if(kind.equals("wall")) {
                var shape=new BoxCollisionShape(new Vector3f(.02f,1,2));shape.setMargin(.001f);var wall=new PhysicsRigidBody(shape,0);wall.setPhysicsLocation(new Vector3f(.35f,.5f,0));wall.setFriction(1);demo.space.add(wall);
                advance(demo,.5,DT);demo.world.driveChassis(new MecanumKinematics.ChassisVelocity(2,0,0),DT);advance(demo,2,DT);double x=demo.world.robotPointWorld(Vector3f.ZERO).x;
                require(x<.19&&x>.02,"Wall failed to block forward drive: x="+x);
                demo.world.driveChassis(new MecanumKinematics.ChassisVelocity(0,0,0),DT);demo.world.chassisBody().applyCentralImpulse(new Vector3f(-3,0,0));advance(demo,.1,DT);
                double retreat=x-demo.world.robotPointWorld(Vector3f.ZERO).x;require(retreat>.01,"Braking canceled push: "+retreat);return Map.of("blocked_x_m",x,"push_retreat_m",retreat);
            }
            if(kind.equals("no-grip"))floor(demo).setFriction(0);
            complete(demo);var r=demo.observations.get(0);
            require("no clear movement".equals(r.get("outcome")),"Expected no movement: "+r);
            if(kind.equals("belly"))require(((Number)r.get("supported_wheels")).intValue()==0&&((Number)r.get("scraping_contacts")).intValue()>0,"Missing belly/no-support diagnosis: "+r);
            return r;
        }
    }
    static Map<String,Object> paddle(int direction) {
        var space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(Vector3f.ZERO);var shaft=new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(.002f,.002f,.012f)),1);shaft.setKinematic(true);space.add(shaft);
            var spec=new FlexibleIntakeConfig(List.of("flap"),3,.9,.025,.05,.003,.015,.5,.8,1,500,.3,new Vector3f(-1,-1,-1),new Vector3f(1,1,1));
            var flap=new FlexibleFlap(space,shaft,new Transform(),.0045,spec);var piece=new PhysicsRigidBody(new SphereCollisionShape(.008f),.003f);piece.setPhysicsLocation(new Vector3f(.044f,0,0));piece.setEnableSleep(false);piece.setFriction(1);space.add(piece);
            for(int i=0;i<240;i++){shaft.setPhysicsRotation(new Quaternion().fromAngleAxis(direction*i/240f*3,Vector3f.UNIT_Z));space.update(1f/240,0);}
            double travel=piece.getPhysicsLocation().y,bend=flap.maxDeflectionRad();require(direction*travel>.01&&bend>.005,"Paddle failed to transport/bend: travel="+travel+" bend="+bend);
            return Map.of("piece_travel_m",travel,"bend_rad",bend,"piece_remains_physical",space.contains(piece));
        }finally{space.destroy();}
    }
    static Map<String,Object> pickup() {
        var space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            var world=new PhysicsWorld(ASSETS,new Node(),space);world.buildChassis(new Node(),3,Vector3f.ZERO);space.setGravity(Vector3f.ZERO);
            var start=new Vector3f(.3f,.2f,0);var end=new Vector3f(.6f,.2f,.1f);world.buildGamePiece(start);world.updateIntake(true,start,.12f);require(world.isPieceHeld(),"Proximity pickup failed");
            world.updateIntake(true,end,.12f);world.updateIntake(false,end,.12f);require(!world.isPieceHeld()&&world.getGamePiecePosition().distance(end)<.001,"Carry/release failed");
            space.update(DT,0);require(space.contains(world.gamePieceBody()),"Released piece missing from native physics");return Map.of("mode","legacy proximity torus; flexible contact tested separately","released",true,"position_m",List.of((double)end.x,(double)end.y,(double)end.z));
        }finally{space.destroy();}
    }
    static Map<String,Object> portable(GuidedSetupController c,Path folder)throws Exception {
        c.wheelGrip("wheel_joint0",.73);var before=c.session.profile("robot");
        try(var prepared=new ModelValidation.Prepared(c.session.modelPath("robot"),ASSETS)){prepared.writeProof();}
        Path saved=Path.of(c.models.run("save",c.session.modelPath("robot").toString(),c.models.library.toString(),"--reviewed").toString());
        Path zip=folder.resolve("portable.zip");c.models.run("export",saved.toString(),zip.toString());
        var imported=new GuidedSetupController(folder.resolve("restored"),null);imported.session.start("robot",null);imported.load("robot","bundle",zip,null);
        require(imported.session.profile("robot").get("runtime").equals(before.get("runtime")),"Portable runtime changed");require(imported.session.ready("robot"),"Reviewed synthetic fixture lost readiness");
        Path changed=SyntheticRobots.zip(folder.resolve("changed-cad.zip"),"robot.urdf",SyntheticRobots.urdf(true,false,false,SyntheticRobots.Dimensions.standard()).replace(".3 .2 .06",".32 .2 .06"));
        imported.load("robot","cad",changed,imported.session.modelPath("robot"));var after=imported.session.profile("robot");
        require(after.get("runtime").equals(before.get("runtime")),"Migration lost drive/contact settings");require(!imported.session.ready("robot"),"Changed CAD bypassed collision review");
        require(FieldPackage.map(after.get("parameters")).equals(FieldPackage.map(before.get("parameters"))),"Migration changed parameters");
        return Map.of("export_import_settings_preserved",true,"migration_settings_preserved",true,"changed_cad_requires_review",true,"wheel_grip",.73);
    }
    static Map<String,Object> stl(Path folder)throws Exception {
        var c=new GuidedSetupController(folder.resolve("library"),null);c.session.start("robot",null);
        String xml="<robot name='Millimeter STL fixture'><link name='body'><visual><geometry><mesh filename='body.stl'/></geometry></visual></link></robot>";
        c.load("robot","fresh",SyntheticRobots.zip(folder.resolve("millimeter-stl.zip"),Map.of("model.urdf",xml,"body.stl",SyntheticRobots.stlBox())),null);c.units("robot","mm");
        var setup=c.motionSetup();var scene=new ImportedRobotScene(setup.urdf(),setup.profile().artifact("robot"),setup.hardware(),ASSETS,8);setup.profile().configure(scene);scene.parts();scene.root.updateGeometricState();
        var box=(com.jme3.bounding.BoundingBox)scene.root.getWorldBound();double x=box.getXExtent()*2,y=box.getYExtent()*2,z=box.getZExtent()*2;
        require(Math.abs(x-.1)<.0001&&Math.abs(y-.06)<.0001&&Math.abs(z-.08)<.0001,"STL units/basis wrong: "+List.of(x,y,z));require(setup.plan().drive.isEmpty(),"STL invented drive actions");return Map.of("source_unit","mm","native_size_xyz_m",List.of(x,y,z),"invented_drive_actions",setup.plan().drive.size());
    }
    static ValidationReport suite(Path folder)throws Exception {
        Files.createDirectories(folder);NativeLibraryLoader.loadNativeLibrary("bulletjme",true);var r=new ValidationReport();
        r.metadata.putAll(Map.of("fixtures",SyntheticRobots.VERSION,"engine","Minie 9.0.3 / Bullet","java",System.getProperty("java.version"),"os",System.getProperty("os.name"),"contact_timestep_s",(double)DT,"motion_timestep_s",(double)RobotMotionDemo.DT,"paddle_timestep_s",1./240));
        // Import inside each check so bad CAD/configuration appears in the report instead of aborting it.
        var d=SyntheticRobots.Dimensions.standard();
        r.check("drive/differential","Four drive directions; no strafe; correct signs",()->movements(SyntheticRobots.importRobot(folder.resolve("differential"),true,false,false,d).motionSetup(),4));
        r.check("drive/mecanum","Six directions including both strafes; correct signs",()->movements(SyntheticRobots.importRobot(folder.resolve("mecanum"),false,false,false,d).motionSetup(),6));
        r.check("mechanisms/arm-slide-servo-intake","Four drive and eight mechanism targets; all reachable within limits",()->movements(SyntheticRobots.importRobot(folder.resolve("mechanisms"),true,true,true,d).motionSetup(),12));
        for(String scenario:List.of("airborne","wall","no-grip","belly"))r.check("contacts/"+scenario,"Native support/grip, blocking or external push matches scenario",()->{
            var c=SyntheticRobots.importRobot(folder.resolve(scenario),true,false,false,d);
            if(scenario.equals("belly")){var p=c.session.profile("robot");var s=FieldPackage.map(FieldPackage.map(FieldPackage.map(p.get("entities")).get("base")).get("settings"));s.put("collision_strategy","box");s.put("box_size_m",List.of(.3,.2,.16));c.update("robot",p);c.compile("robot");}
            return contactScenario(c.motionSetup(),scenario);
        });
        r.check("intake/contact-forward","Flexible native paddles bend and transport the piece inward",()->paddle(1));
        r.check("intake/contact-reverse","Reversed paddles reverse native piece transport",()->paddle(-1));
        r.check("intake/pickup-carry-release","Legacy proximity capture carries and releases a native torus",SimulatorValidation::pickup);
        r.check("import/stl-units","100 × 80 × 60 mm becomes 0.1 × 0.08 × 0.06 m without invented actuators",()->stl(folder.resolve("stl")));
        r.check("import/portable-migration","Drive/grip settings survive export, import and changed CAD; renewed review required",()->portable(SyntheticRobots.importRobot(folder.resolve("portable"),true,false,false,d),folder.resolve("portable")));
        return r;
    }
    public static void main(String[] args)throws Exception {
        Path output=Path.of("build/simulator-validation"),profile=null,project=null;
        for(int i=0;i<args.length;i++){if(i+1>=args.length)throw new IllegalArgumentException("Each option needs a value: --output, --profile, --project");switch(args[i]){case "--output"->output=Path.of(args[++i]);case "--profile"->profile=Path.of(args[++i]);case "--project"->project=Path.of(args[++i]);default->throw new IllegalArgumentException("Unknown option: "+args[i]);}}
        if(project!=null&&profile==null)throw new IllegalArgumentException("--project requires --profile");
        if(profile!=null&&output.toAbsolutePath().normalize().startsWith(profile.toRealPath().getParent()))throw new IllegalArgumentException("Choose an output outside the model directory");
        Path folder=Files.createDirectories(output.toAbsolutePath().normalize()).resolve("run-"+UUID.randomUUID());Files.createDirectories(folder);
        ValidationReport report;
        if(profile==null)report=suite(folder);
        else {NativeLibraryLoader.loadNativeLibrary("bulletjme",true);report=new ValidationReport();report.metadata.put("profile",profile.toAbsolutePath().toString());report.metadata.put("motion_timestep_s",(double)RobotMotionDemo.DT);Path p=profile,team=project;report.check("profile/configured-movements","Every configured motion occurs in its declared direction within joint limits",()->{var setup=RobotMotionDemo.setup(p,team);return movements(setup,setup.plan().items().size());});}
        report.metadata.putAll(Map.of("engine","Minie 9.0.3 / Bullet","java",System.getProperty("java.version"),"os",System.getProperty("os.name")));
        report.write(folder);System.out.println("[VALIDATION] "+(report.passed()?"PASS":"FAIL")+" "+report.cases.size()+" scenarios; "+folder.resolve("report.md"));
        if(!report.passed())throw new IllegalStateException("Simulator validation failed. Inspect "+folder.resolve("report.json"));
    }
}
