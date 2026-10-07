package simrunner;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.system.NativeLibraryLoader;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import physics.MotorSpec;
import simcore.ElectricalConfig;
import simcore.MiniJson;
import simcore.RobotUrdf;
import simcore.SimDcMotorEx;

import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

/** Transmission efficiency and backlash through the native articulated-joint physics. */
class ActuatorPhysicsTest {
    @TempDir Path temp;
    @BeforeAll static void nativePhysics() { NativeLibraryLoader.loadNativeLibrary("bulletjme", true); }
    private static final float DT = 1f / 60;

    private static String link(String name, String center, double mass) {
        return "<link name='" + name + "'><inertial><origin xyz='" + center + "'/><mass value='" + mass
            + "'/><inertia ixx='.01' iyy='.02' izz='.02' ixy='0' ixz='0' iyz='0'/></inertial>"
            + "<collision><origin xyz='" + center + "'/><geometry><box size='.1 .1 .1'/></geometry></collision></link>";
    }

    private record Result(double jointM, double encoderTicks) { }

    /** Drives a slide with the given motor tuning for a third of a second, then reports where the joint and encoder ended up. */
    private Result run(String tuning, double power) throws Exception {
        Path urdf = temp.resolve("slide-" + tuning.hashCode() + ".urdf");
        Files.writeString(urdf, "<robot name='slide'>" + link("base", "0 0 0", 5) + link("carriage", ".1 0 0", 1)
            + "<joint name='slide' type='prismatic'><parent link='base'/><child link='carriage'/><origin xyz='.2 0 0'/>"
            + "<axis xyz='1 0 0'/><limit lower='-.5' upper='.5' effort='1000'/></joint>"
            + "<transmission name='tx'><joint name='slide'/><actuator name='motor'><mechanicalReduction>10</mechanicalReduction></actuator></transmission></robot>");
        PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(Vector3f.ZERO);
            HardwareMap map = new HardwareMap();
            SimDcMotorEx motor = new SimDcMotorEx("motor", new MotorSpec("test", 1, 2, 9, 30, 12, 500));
            motor.applyTuning(ElectricalConfig.parse(MiniJson.parseObject("{\"motor_defaults\":" + tuning + "}")).motorDefaults);
            map.register("motor", motor);
            ImportedRobotScene scene = new ImportedRobotScene(RobotUrdf.parse(urdf), urdf, map, new DesktopAssetManager(true), 8);
            PhysicsWorld world = new PhysicsWorld(null, new Node(), space);
            ArticulatedRobot robot = new ArticulatedRobot(scene, world, new Node(), Vector3f.ZERO);
            world.chassisBody().setMass(0);
            motor.setPower(power);
            for (int i = 0; i < 20; i++) {
                space.update(DT, 0);
                motor.integrate(12, DT, Math.round(i * DT * 1000));
            }
            return new Result(robot.jointPosition("slide"), motor.getRawTicks());
        } finally { space.destroy(); }
    }

    @Test void lowerTransmissionEfficiencyMovesTheSlideLessForTheSameCommand() throws Exception {
        Result ideal = run("{}", 0.3);
        Result lossy = run("{\"transmission_efficiency\":0.4}", 0.3);
        assertTrue(ideal.jointM() > 0.02, "baseline slide moves: " + ideal);
        assertTrue(lossy.jointM() < ideal.jointM() * 0.8, "lossy " + lossy + " vs ideal " + ideal);
        assertTrue(lossy.jointM() > 0, "still moves in the right direction");
    }

    @Test void backlashLeavesTheEncoderBehindTheJointByHalfTheGapAtMotorSideScale() throws Exception {
        Result tight = run("{\"viscous_friction_nm_s_per_rad\":0.0}", 0.3);
        Result loose = run("{\"backlash_motor_rad\":0.4}", 0.3);
        assertEquals(tight.jointM(), loose.jointM(), tight.jointM() * 0.02 + 1e-4, "backlash here only affects what the encoder reports");
        double gapTicks = 0.2 * 500 / (2 * Math.PI);
        assertEquals(tight.encoderTicks() - gapTicks, loose.encoderTicks(), 1.5);
    }
}
