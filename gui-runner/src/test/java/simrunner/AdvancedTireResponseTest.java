package simrunner;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.Path;
import java.util.*;
import com.jme3.math.Vector3f;
import simcore.SimDcMotorEx;
import static org.junit.jupiter.api.Assertions.*;
class AdvancedTireResponseTest {
    @TempDir Path tmp;
    @BeforeAll static void natives(){com.jme3.system.NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    @Test void nativeTransientTiresRespectMaterialsAndShaftReactionAtBothTimesteps()throws Exception {
        var c=SyntheticRobots.importRobot(tmp,true,false,false,SyntheticRobots.Dimensions.standard());var p=c.session.profile("robot");
        var spec=Map.of("static_mu",.9,"sliding_mu",.6,"lateral_scale",1.,"stiffness_n_per_mps",100.,"transition_mps",.15);
        var values=new LinkedHashMap<String,Object>(Map.of("traction",spec,"omni",spec,"omni_joints",List.of(),"reflected_motor_inertia_kg_m2",.0015,"contact_tolerance_m",.004));
        values.put("traction_response",Map.of("lateral_stiffness_n_per_mps",70.,"lateral_mu",.4,"relaxation_time_s",.08));FieldPackage.map(p.get("runtime")).put("tires",values);c.update("robot",p);c.compile("robot");
        double previous=0;
        for(int hz:new int[]{120,480,960})try(var d=new RobotMotionDemo(c.motionSetup(),new com.jme3.asset.DesktopAssetManager(true),"drive/forward")) {
            float dt=1f/hz;d.space.setAccuracy(dt);
            // Isolate transient traction convergence from unconstrained yaw/contact bifurcations.
            // Free turning and asymmetric support are exercised in the native validation matrix.
            d.world.chassisBody().setAngularFactor(Vector3f.ZERO);SimulatorValidation.advance(d,.6,dt);var motors=d.setup.hardware().getAll(SimDcMotorEx.class);
            d.setup.hardware().get(SimDcMotorEx.class,"leftDrive").setPower(-.5);d.setup.hardware().get(SimDcMotorEx.class,"rightDrive").setPower(.5);
            var start=d.world.getChassisPosition();d.world.chassisBody().setLinearVelocity(new Vector3f(0,0,.1f));
            double load=0,forces=0;int supported=0;
            for(int i=0;i<hz;i++){for(var m:motors)m.integrate(12,dt,Math.round(i*1000./hz));d.space.update(dt,0);
                for(var w:d.world.tireDrive().states()){load+=w.normalN();forces+=w.forceN();if(w.supported())supported++;assertTrue(Double.isFinite(w.slipMps()));assertTrue(Math.hypot(w.forceN(),w.lateralN())<=w.gripN()+1e-5);}
                for(var r:d.world.tireDrive().reactions())assertEquals(-r.contactTorqueNm(),r.shaftTorqueNm(),1e-8);
            }
            double travel=d.world.getChassisPosition().x-start.x;System.out.println("[TRANSIENT] hz="+hz+" travel="+travel+" mean normal="+load/hz+" mean force="+forces/hz+" supported="+supported/(double)hz);assertTrue(travel>.3&&travel<2);if(previous>0)assertEquals(previous,travel,.05);previous=travel;
            SimulatorValidation.floor(d).setFriction(0);for(int i=0;i<10;i++)d.space.update(dt,0);assertTrue(d.world.tireDrive().states().stream().allMatch(w->Math.abs(w.forceN())<1e-8&&Math.abs(w.lateralN())<1e-8));
        }
        values.put("traction_response",Map.of("lateral_mu",.4));assertThrows(IllegalArgumentException.class,()->TireDriveConfig.parse(values));
    }
}
