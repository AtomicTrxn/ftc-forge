package simrunner;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.*;
import com.jme3.scene.Node;
import com.jme3.system.NativeLibraryLoader;
import com.qualcomm.robotcore.hardware.DcMotor;
import simcore.*;
import java.nio.file.Path;

/** Fixed-step capture/carry/release diagnostics using the team's prepared CAD. */
public final class TorusRetentionProbe {
    static final float DT=1f/480;
    final PhysicsSpace space;
    final PhysicsWorld world;
    final ArticulatedRobot robot;
    final com.qualcomm.robotcore.hardware.HardwareMap map;
    final SimConfig config;
    final SimDcMotorEx left,right,intake;
    double time;
    TorusRetentionProbe(Path project) throws Exception {
        config=SimConfig.load(project);
        if(config.tires==null||config.flexibleIntake==null||config.torusRetention==null)throw new IllegalArgumentException("Probe needs tire, flexible intake and torus_retention models");
        space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        var assets=new DesktopAssetManager(true);var field=new Node();
        world=new PhysicsWorld(assets,field,space);world.buildFieldBoundary();
        map=HardwareMapBuilder.build(RobotConfigXml.parse(project.resolve(config.robotConfig).toFile()),PresetRobotConfig.load(project.resolve(config.presetMotors)));
        var urdf=RobotUrdf.parse(project.resolve(config.urdf));
        if(config.totalMassKg!=null)urdf=urdf.withTotalMassKg(config.totalMassKg);
        var scene=new ImportedRobotScene(urdf,project.resolve(config.urdf),map,assets,config.vhacdMaxHulls,java.util.Set.copyOf(config.drive.motorNames()));
        if(config.robotProfile!=null)config.robotProfile.configure(scene);
        scene.tireContacts=true;scene.flexibleIntake=config.flexibleIntake;
        scene.collisionOmissions=config.collisionOmissions;
        robot=new ArticulatedRobot(scene,world,field,new Vector3f(0,(float)config.startHeightM,0));
        world.installTires(new TireDrive(world,map,scene,config.drive,config.tires));
        world.installFlexibleIntake(new FlexibleIntake(world,scene,robot,config.flexibleIntake));
            if (config.torusRetention != null) world.flexibleIntake().installRetention(robot,map,config.intake,config.torusRetention);
        left=map.get(SimDcMotorEx.class,config.drive.leftMotor());right=map.get(SimDcMotorEx.class,config.drive.rightMotor());intake=map.get(SimDcMotorEx.class,config.intake.motor());
        left.setDirection(DcMotor.Direction.REVERSE);left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        world.buildGamePiece(config.gamePieceStart==null ? new Vector3f(.42f,.034f,0) : config.gamePieceStart);
        world.gamePieceBody().setPhysicsRotation(new Quaternion().fromAngleAxis(FastMath.HALF_PI,Vector3f.UNIT_X));
    }
    Vector3f localPiece(){return world.getChassisRotation().inverse().mult(world.getGamePiecePosition().subtract(world.robotPointWorld(Vector3f.ZERO)));}
    void step(){for(var motor:map.getAll(SimDcMotorEx.class))motor.integrate(12,DT,Math.round(time*1000));space.update(DT,0);time+=DT;world.updateIntake(config.intake.active(map),world.robotPointWorld(config.intake.point()),config.intake.captureRadiusM());}
    TorusRetention grip(){return world.flexibleIntake().retention;}
    void require(boolean condition,String message){if(!condition)throw new IllegalStateException(message+" at t="+time+" local="+localPiece()+" state="+grip().state());}
    boolean phase(String name,double duration,double l,double r,double in,boolean mustRetain) {
        left.setPower(l);right.setPower(r);intake.setPower(in);
        boolean sustained=true;
        for(int i=0;i<Math.round(duration/DT);i++) {
            step();
            Vector3f p=world.getGamePiecePosition();
            require(space.contains(world.gamePieceBody())&&world.gamePieceBody().isDynamic(),"Piece lost native dynamics");
            require(Float.isFinite(p.x)&&Float.isFinite(p.y)&&Float.isFinite(p.z)&&p.y>-.01&&Math.abs(p.x)<1.8&&Math.abs(p.z)<1.8,"Piece escaped field");
            require(world.gamePieceBody().getLinearVelocity().length()<3,"Unstable piece speed");
            if(i>=Math.round((duration-.5)/DT)) sustained &= world.isPieceHeld();
            if(mustRetain) require(world.isPieceHeld(),"Lost retention during "+name);
        }
        System.out.println("[RETENTION] t="+time+" phase="+name+" local="+localPiece()+" contained="+world.isPieceHeld()+" state="+grip().state()+" anchor="+grip().anchor()+" normal="+world.gamePieceBody().getPhysicsRotation().mult(Vector3f.UNIT_Z)+" omega="+intake.getOmegaRadS());
        return sustained;
    }
    void cycle(String label,double power,double drive) {
        phase(label+" approach",1.2,drive,drive,power,false);
        require(phase(label+" pickup",4,0,0,power,false),"Capture was not sustained for 0.5 s");
        Vector3f seated=localPiece();Vector3f chassisStart=world.getChassisPosition();
        phase(label+" carry",1,.12,.12,power,true);
        require(world.getChassisPosition().distance(chassisStart)>.1,"Carry did not move chassis");
        Vector3f beforeTurn=world.getChassisRotation().mult(Vector3f.UNIT_X);
        phase(label+" turn",1,-.12,.12,power,true);
        require(beforeTurn.angleBetween(world.getChassisRotation().mult(Vector3f.UNIT_X))>.25,"Carry did not turn chassis");
        phase(label+" stopped",2,0,0,0,true);
        require(localPiece().distance(seated)<.04,"Excessive carried/stopped drift");
        phase(label+" reverse",2,0,0,-power,false);
        require(!world.isPieceHeld() && grip().state()==TorusRetention.State.FREE && localPiece().x>.3,"Reverse did not release outward");
        phase(label+" released idle",.5,0,0,0,false);
        require(grip().state()==TorusRetention.State.FREE,"Released piece immediately reacquired");
        require(grip().peakForceN<=config.torusRetention.maxForceN()+1e-5,"Force cap exceeded");
        require(grip().peakTorqueNm<=config.torusRetention.maxTorqueNm()+1e-5,"Torque cap exceeded");
        require(grip().peakLoadNm>0,"Draw did not load the physical shaft");
        require(Math.abs(robot.jointPosition("intake_upper_joint")-robot.jointPosition("intake_lower_joint"))<.01,"Chain lost phase");
    }
    public static void main(String[] args)throws Exception {
        if(args.length!=1)throw new IllegalArgumentException("Usage: TorusRetentionProbe <preparedProject>");
        NativeLibraryLoader.loadNativeLibrary("bulletjme",true);
        Path project=Path.of(args[0]);
        for(int scenario=0;scenario<3;scenario++) {
            var p=new TorusRetentionProbe(project);
            try {
                p.require(p.config.torusRetention!=null,"Probe requires torus_retention");
                float offset=new float[]{0,.02f,-.02f}[scenario];
                p.world.gamePieceBody().setPhysicsLocation(new Vector3f(.42f,.034f,offset));
                p.phase("phase variation "+scenario,new double[]{.05,.23,.41}[scenario],0,0,1,false);
                p.require(p.grip().acquisitions==0,"Uncontacted nearby piece acquired");
                if(scenario==1)p.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                p.cycle("case "+scenario,new double[]{1,.7,1}[scenario],new double[]{.1,.1,.12}[scenario]);
                if(scenario==0) {
                    p.cycle("second cycle",1,.1);
                    p.require(p.grip().acquisitions==2&&p.grip().releases==2,"Repeat capture/release counts wrong");
                }
                System.out.println("[RETENTION PASS] case="+scenario+" acquisitions="+p.grip().acquisitions+" releases="+p.grip().releases+" maxForceN="+p.grip().peakForceN+" maxTorqueNm="+p.grip().peakTorqueNm+" peakShaftLoadNm="+p.grip().peakLoadNm);
            }finally{p.space.destroy();}
        }
        for(int scenario=0;scenario<4;scenario++) {
            var p=new TorusRetentionProbe(project);
            try {
                p.world.gamePieceBody().setPhysicsLocation(new Vector3f(.265f,.034f,0));
                if(scenario==2){p.world.gamePieceBody().setPhysicsLocation(new Vector3f(.42f,.034f,0));p.phase("spin",.2,0,0,1,false);p.world.gamePieceBody().setPhysicsLocation(new Vector3f(.265f,.034f,0));}
                if(scenario==3)p.robot.bodyForLink("intake_upper").setMass(0);
                p.phase("inactive/reverse/stalled "+scenario,2,0,0,scenario==1?-1:scenario==3?1:0,false);
                if(scenario==3)p.require(Math.abs(p.intake.getOmegaRadS())<.1,"Stall fixture was not blocked");
                p.require(p.grip().acquisitions==0&&!p.world.isPieceHeld(),"Inactive/reverse intake acquired");
            }finally{p.space.destroy();}
        }
        var p=new TorusRetentionProbe(project);
        try {
            p.phase("overload approach",1.2,.1,.1,1,false);
            p.require(p.phase("overload pickup",4,0,0,1,false),"Overload setup failed");
            p.intake.setPower(0);
            Vector3f outward=p.world.getChassisRotation().mult(Vector3f.UNIT_X).mult(20);
            for(int i=0;i<240 && p.grip().state()!=TorusRetention.State.FREE;i++){p.world.gamePieceBody().applyCentralForce(outward);p.step();}
            p.require(p.grip().state()==TorusRetention.State.FREE&&!p.world.isPieceHeld(),"Finite grip failed to break under overload");
            Vector3f released=p.world.getGamePiecePosition();
            p.require(p.space.contains(p.world.gamePieceBody())&&p.world.gamePieceBody().isDynamic()
                &&Float.isFinite(released.x)&&Float.isFinite(released.y)&&Float.isFinite(released.z),"Overload lost finite native dynamics");
            System.out.println("[RETENTION PASS] inactive, reverse, coasting, stalled and overload gates");
        }finally{p.space.destroy();}
    }
}
