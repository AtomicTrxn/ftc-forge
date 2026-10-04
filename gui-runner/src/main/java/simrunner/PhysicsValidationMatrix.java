package simrunner;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.collision.*;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;
import physics.MecanumKinematics;
import simcore.SimDcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import java.nio.file.*;
import java.util.*;
import static simrunner.ValidationReport.require;

/** Parameter sweeps of actual native dynamics with analytic bounds and timestep comparisons. */
final class PhysicsValidationMatrix {
    static String number(double x){return Double.toString(x);}
    static void stable(RobotMotionDemo demo) {
        for(var b:demo.space.getRigidBodyList())if(b.isDynamic())require(Vector3f.isValidVector(b.getPhysicsLocation())&&Vector3f.isValidVector(b.getLinearVelocity())&&Vector3f.isValidVector(b.getAngularVelocity())&&b.getPhysicsLocation().length()<50&&b.getLinearVelocity().length()<50&&b.getAngularVelocity().length()<100,"Nonfinite or explosive native state");
    }
    static double penetration(RobotMotionDemo demo) {
        double depth=0;for(long m:demo.space.listManifoldIds())for(long p:PersistentManifolds.listPointIds(m))depth=Math.max(depth,-ManifoldPoints.getDistance1(p));return depth;
    }
    static Map<String,Object> drive(RobotMotionDemo.Setup setup,double friction,float dt,ValidationMatrixConfig cfg)throws Exception {
        try(var demo=new RobotMotionDemo(setup,new DesktopAssetManager(true),"drive/forward")) {
            SimulatorValidation.floor(demo).setFriction((float)friction);SimulatorValidation.advance(demo,cfg.get("settle_s"),dt);
            var body=demo.world.chassisBody();double mass=demo.space.getRigidBodyList().stream().filter(PhysicsRigidBody::isDynamic).mapToDouble(PhysicsRigidBody::getMass).sum();
            require(Math.abs(mass-setup.urdf().totalMassKg())<=cfg.get("mass_tolerance_kg"),"Mass counted more than once: "+mass);
            var start=body.getPhysicsLocation();SimulatorValidation.advance(demo,.5,dt);double drift=start.distance(body.getPhysicsLocation());
            require(drift<=cfg.get("drift_tolerance_m"),"Zero-command drift: "+drift+" m");
            start=body.getPhysicsLocation();double initialSpeed=body.getLinearVelocity().x,maxForce=0,normalSum=0,depth=0;
            int ticks=(int)Math.round(cfg.get("drive_s")/dt);demo.world.driveChassis(new MecanumKinematics.ChassisVelocity(5,0,0),dt);
            for(int i=0;i<ticks;i++) {
                demo.space.update(dt,0);stable(demo);var impulse=demo.world.driveImpulse();
                double planar=Math.hypot(impulse.xNs(),impulse.zNs());double allowance=cfg.get("impulse_tolerance_ns");
                require(planar<=impulse.availableNs()+allowance,"Drive exceeds material grip: "+planar+" > "+impulse.availableNs());
                if(impulse.availableNs()>allowance&&impulse.availableNms()>allowance)require(Math.hypot(planar/impulse.availableNs(),impulse.yawNms()/impulse.availableNms())<=1+allowance,"Combined drive/yaw grip exceeded");
                maxForce=Math.max(maxForce,planar/dt);normalSum+=impulse.normalNs();depth=Math.max(depth,penetration(demo));
            }
            double seconds=ticks*dt,speed=body.getLinearVelocity().x,travel=body.getPhysicsLocation().x-start.x;
            double analyticBound=.6*friction*9.81*seconds+cfg.get("velocity_bound_tolerance_mps");
            require(speed-initialSpeed<=analyticBound,"Speed exceeds mu*g*t bound: "+speed+" > "+analyticBound);
            require(depth<=cfg.get("penetration_tolerance_m"),"Excessive native penetration: "+depth);
            if(friction==0)require(Math.abs(travel)<=cfg.get("drift_tolerance_m")&&maxForce<=cfg.get("impulse_tolerance_ns")/dt,"Zero-grip drive moved");
            else require(travel>0&&speed>0,"Supported drive failed to advance");
            // Zero target is a passive brake; it must dissipate, never inject horizontal drive work.
            demo.world.driveChassis(new MecanumKinematics.ChassisVelocity(0,0,0),dt);double energyBefore=.5*mass*body.getLinearVelocity().lengthSquared(),positiveWork=0;
            for(int i=0;i<Math.round(.5/dt);i++){var velocity=body.getLinearVelocity();demo.space.update(dt,0);stable(demo);var impulse=demo.world.driveImpulse();positiveWork+=Math.max(0,impulse.xNs()*velocity.x+impulse.zNs()*velocity.z);}
            double energyAfter=.5*mass*body.getLinearVelocity().lengthSquared();require(energyAfter<=energyBefore+cfg.get("energy_increase_tolerance_j")&&positiveWork<=cfg.get("energy_increase_tolerance_j"),"Unpowered brake injected energy: before="+energyBefore+" after="+energyAfter+" positive work="+positiveWork);
            return Map.ofEntries(Map.entry("mass_kg",mass),Map.entry("floor_friction",friction),Map.entry("timestep_s",(double)dt),Map.entry("zero_command_drift_m",drift),Map.entry("travel_m",travel),Map.entry("speed_mps",speed),Map.entry("analytic_speed_gain_bound_mps",analyticBound),Map.entry("max_drive_force_n",maxForce),Map.entry("mean_support_n",normalSum/seconds),Map.entry("max_penetration_m",depth),Map.entry("brake_energy_before_j",energyBefore),Map.entry("brake_energy_after_j",energyAfter),Map.entry("positive_brake_work_j",positiveWork));
        }
    }
    static Map<String,Object> convergence(List<Map<String,Object>> rows,int expected,ValidationMatrixConfig cfg) {
        require(rows.size()==expected,"A timestep failed; cannot establish convergence");
        double low=rows.stream().mapToDouble(r->FieldPackage.num(r,"travel_m")).min().orElseThrow(),high=rows.stream().mapToDouble(r->FieldPackage.num(r,"travel_m")).max().orElseThrow();
        double tolerance=cfg.get("convergence_absolute_m")+cfg.get("convergence_relative")*Math.max(Math.abs(low),Math.abs(high));
        require(high-low<=tolerance,"Travel spread "+(high-low)+" m exceeds "+tolerance+" m");return Map.of("travel_min_m",low,"travel_max_m",high,"allowed_spread_m",tolerance,"timesteps_s",rows.stream().map(r->r.get("timestep_s")).toList());
    }
    static Map<String,Object> tires(RobotMotionDemo.Setup setup,String mode,float dt,ValidationMatrixConfig cfg)throws Exception {
        try(var demo=new RobotMotionDemo(setup,new DesktopAssetManager(true),"drive/forward")) {
            if(mode.equals("zero-grip"))SimulatorValidation.floor(demo).setFriction(0);
            SimulatorValidation.advance(demo,cfg.get("settle_s"),dt);
            if(mode.equals("wall")){var shape=new com.jme3.bullet.collision.shapes.BoxCollisionShape(new Vector3f(.02f,1,2));shape.setMargin(.001f);var wall=new PhysicsRigidBody(shape,0);wall.setPhysicsLocation(new Vector3f(.35f,.5f,0));demo.space.add(wall);}
            if(mode.equals("airborne")){demo.space.setGravity(Vector3f.ZERO);var b=demo.world.chassisBody();b.setPhysicsLocation(b.getPhysicsLocation().add(0,2,0));}
            var start=demo.world.getChassisPosition();double reactionError=0,maxSlip=0,peakCurrent=0,omegaSum=0;
            int ticks=Math.round(2/dt);var motors=setup.hardware().getAll(SimDcMotorEx.class);
            setup.hardware().get(SimDcMotorEx.class,"leftDrive").setPower(-.7);setup.hardware().get(SimDcMotorEx.class,"rightDrive").setPower(.7);
            for(int i=0;i<ticks;i++) {
                for(var m:motors)m.integrate(12,dt,Math.round(i*dt*1000));demo.space.update(dt,0);stable(demo);
                for(var state:demo.world.tireDrive().states()){maxSlip=Math.max(maxSlip,Math.abs(state.slipMps()));require(Math.hypot(state.forceN(),state.lateralN())<=state.gripN()+cfg.get("impulse_tolerance_ns")/dt,"Tire force exceeds load/material grip");if(mode.equals("airborne")||mode.equals("zero-grip"))require(Math.abs(state.forceN())<=cfg.get("impulse_tolerance_ns"),"Unsupported/zero-grip tire force");}
                for(var reaction:demo.world.tireDrive().reactions())reactionError=Math.max(reactionError,Math.abs(reaction.contactTorqueNm()+reaction.shaftTorqueNm()));
                peakCurrent=Math.max(peakCurrent,motors.stream().mapToDouble(m->m.getCurrent(CurrentUnit.AMPS)).max().orElse(0));
                omegaSum+=motors.stream().mapToDouble(m->Math.abs(m.getOmegaRadS())).average().orElseThrow();
            }
            double travel=demo.world.getChassisPosition().x-start.x,omega=motors.stream().mapToDouble(m->Math.abs(m.getOmegaRadS())).average().orElseThrow();
            require(reactionError<=cfg.get("tire_reaction_tolerance_nm"),"Unequal contact/shaft reaction: "+reactionError);
            if(mode.equals("wall"))require(travel<.19,"Tires crossed wall: "+travel);
            if(mode.equals("airborne")||mode.equals("zero-grip"))require(Math.abs(travel)<cfg.get("drift_tolerance_m")&&omega>1,"Free tires must spin without driving robot: "+travel+" / "+omega);
            if(mode.equals("free"))require(travel>.05,"Supported tires did not drive robot");
            return Map.of("mode",mode,"timestep_s",(double)dt,"travel_m",travel,"shaft_speed_rad_s",omega,"mean_shaft_speed_rad_s",omegaSum/ticks,"final_current_a",motors.stream().mapToDouble(m->m.getCurrent(CurrentUnit.AMPS)).average().orElseThrow(),"peak_current_a",peakCurrent,"max_slip_mps",maxSlip,"max_reaction_error_nm",reactionError);
        }
    }
    static void append(ValidationReport r,Path folder,ValidationMatrixConfig cfg)throws Exception {
        Files.createDirectories(folder);r.metadata.put("matrix_version","native-matrix-v1");r.metadata.put("matrix_configuration",cfg.values);
        int model=0;
        for(double mass:cfg.list("mass_kg"))for(double radius:cfg.list("wheel_radius_m"))for(double track:cfg.list("track_width_m")) {
            String id="matrix/mass-"+number(mass)+"/radius-"+number(radius)+"/track-"+number(track);
            GuidedSetupController c;
            try{c=SyntheticRobots.importRobot(folder.resolve("drive-"+model++),true,false,false,new SyntheticRobots.Dimensions(mass,radius,track,1));}
            catch(Exception e){r.check(id+"/import","Valid generated dimensions and package",()->{throw e;});continue;}
            for(double mu:cfg.list("floor_friction")) {
                var samples=new ArrayList<Map<String,Object>>();
                for(double dt:cfg.list("timestep_s"))r.check(id+"/grip-"+number(mu)+"/dt-"+number(dt),"Mass once, bounded material force, finite motion, passive brake, bounded penetration and drift",()->{var values=drive(c.motionSetup(),mu,(float)dt,cfg);samples.add(values);return values;});
                r.check(id+"/grip-"+number(mu)+"/convergence","All timesteps pass and travel spread fits the declared tolerance",()->convergence(samples,cfg.list("timestep_s").size(),cfg));
            }
        }
        for(double gear:cfg.list("mechanism_reduction"))r.check("matrix/mechanism-gearing-"+number(gear),"Arm/slide/servo movement and limits survive signed shaft-to-joint reduction",()->SimulatorValidation.movements(SyntheticRobots.importRobot(folder.resolve("gearing-"+number(gear)),true,true,false,new SyntheticRobots.Dimensions(3,.045,.26,gear)).motionSetup(),10));
        var tireModel=SyntheticRobots.importRobot(folder.resolve("tires"),true,false,false,SyntheticRobots.Dimensions.standard());var p=tireModel.session.profile("robot");
        var spec=Map.of("static_mu",.9,"sliding_mu",.6,"lateral_scale",1.,"stiffness_n_per_mps",100.,"transition_mps",.15);FieldPackage.map(p.get("runtime")).put("tires",Map.of("traction",spec,"omni",spec,"omni_joints",List.of(),"reflected_motor_inertia_kg_m2",.0015,"contact_tolerance_m",.004));tireModel.update("robot",p);tireModel.compile("robot");
        for(double dt:cfg.list("timestep_s")) {
            var samples=new HashMap<String,Map<String,Object>>();for(String mode:List.of("free","wall","airborne","zero-grip"))r.check("matrix/tires/"+mode+"/dt-"+number(dt),"Native tire support, bounded shaft reaction, slip and wall loading",()->{var values=tires(tireModel.motionSetup(),mode,(float)dt,cfg);samples.put(mode,values);return values;});
            r.check("matrix/tires/load-comparison/dt-"+number(dt),"Wall lowers shaft speed and raises current; unsupported tires accelerate faster",()->{require(samples.size()==4,"Missing tire scenario");var free=samples.get("free");var wall=samples.get("wall");var air=samples.get("airborne");require(FieldPackage.num(wall,"shaft_speed_rad_s")<FieldPackage.num(free,"shaft_speed_rad_s")&&FieldPackage.num(wall,"final_current_a")>FieldPackage.num(free,"final_current_a")+.1&&FieldPackage.num(air,"mean_shaft_speed_rad_s")>FieldPackage.num(free,"mean_shaft_speed_rad_s")+.1,"Motor speed/current does not reflect contact load: "+samples);return Map.of("free",free,"wall",wall,"airborne",air);});
        }
    }
}
