package simrunner;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.Path;
import java.util.*;
import static org.junit.jupiter.api.Assertions.*;

class PhysicsValidationMatrixTest {
    @TempDir Path tmp;
    @Test void defaultMatrixExercisesAllDimensionsAndTimestepsWithNativeBounds()throws Exception {
        com.jme3.system.NativeLibraryLoader.loadNativeLibrary("bulletjme",true);var r=new ValidationReport();
        PhysicsValidationMatrix.append(r,tmp,ValidationMatrixConfig.load(null));assertEquals(162,r.cases.size());assertTrue(r.passed(),ProfileIO.json(r.data()));
        assertEquals(108,r.cases.stream().filter(c->c.get("id").toString().startsWith("matrix/mass-")&&c.get("id").toString().contains("/dt-")).count());
        assertEquals(36,r.cases.stream().filter(c->c.get("id").toString().endsWith("/convergence")).count());
    }
    @Test void invalidAndUnboundedConfigurationCannotRunOrHideCoverage()throws Exception {
        var base=ValidationMatrixConfig.load(null).values;
        for(var replacement:List.of(Map.entry("mass_kg",(Object)List.of(-1)),Map.entry("timestep_s",(Object)List.of(1./120)),Map.entry("floor_friction",(Object)List.of(.6,.6)),Map.entry("wheel_radius_m",(Object)List.of(Double.NaN)),Map.entry("mechanism_reduction",(Object)List.of(0)),Map.entry("energy_increase_tolerance_j",(Object).1))) {
            var data=new LinkedHashMap<>(base);data.put(replacement.getKey(),replacement.getValue());assertThrows(IllegalArgumentException.class,()->new ValidationMatrixConfig(data));
        }
        var extra=new LinkedHashMap<>(base);extra.put("ignored",true);assertThrows(IllegalArgumentException.class,()->new ValidationMatrixConfig(extra));
        var budget=new LinkedHashMap<>(base);for(String key:List.of("mass_kg","floor_friction","track_width_m"))budget.put(key,key.equals("mass_kg")?List.of(1,2,3,4):key.equals("floor_friction")?List.of(.1,.2,.3,.4):List.of(.24,.3,.4,.5));assertThrows(IllegalArgumentException.class,()->new ValidationMatrixConfig(budget));
        assertThrows(IllegalArgumentException.class,()->SimulatorValidation.main(new String[]{"--matrix-config","unused.json"}));
    }
    @Test void excessiveTimestepSpreadOrMissingSamplesFailConvergence()throws Exception {
        var cfg=ValidationMatrixConfig.load(null);var rows=List.<Map<String,Object>>of(Map.of("travel_m",.1,"timestep_s",1./30),Map.of("travel_m",.3,"timestep_s",1./480));
        assertThrows(IllegalStateException.class,()->PhysicsValidationMatrix.convergence(rows,2,cfg));assertThrows(IllegalStateException.class,()->PhysicsValidationMatrix.convergence(rows,3,cfg));
        assertNotNull(PhysicsValidationMatrix.convergence(List.of(rows.get(0),Map.of("travel_m",.11,"timestep_s",1./480)),2,cfg));
    }
    @Test void teleportedSettledChassisCannotReuseCachedFloorSupport()throws Exception {
        com.jme3.system.NativeLibraryLoader.loadNativeLibrary("bulletjme",true);var c=SyntheticRobots.importRobot(tmp,true,false,false,SyntheticRobots.Dimensions.standard());
        try(var demo=new RobotMotionDemo(c.motionSetup(),new com.jme3.asset.DesktopAssetManager(true),"drive/forward")) {
            SimulatorValidation.advance(demo,.6,1f/120);assertFalse(demo.world.driveContacts().refresh().wheels().isEmpty());
            demo.space.setGravity(com.jme3.math.Vector3f.ZERO);var b=demo.world.chassisBody();b.setPhysicsLocation(b.getPhysicsLocation().add(0,2,0));b.setLinearVelocity(com.jme3.math.Vector3f.ZERO);b.setAngularVelocity(com.jme3.math.Vector3f.ZERO);
            assertTrue(demo.world.driveContacts().refresh().wheels().isEmpty());demo.world.driveChassis(new physics.MecanumKinematics.ChassisVelocity(5,0,1),1f/120);demo.space.update(1f/120,0);
            assertEquals(0,b.getLinearVelocity().length(),.0001);assertEquals(0,b.getAngularVelocity().length(),.0001);
        }
    }
}
