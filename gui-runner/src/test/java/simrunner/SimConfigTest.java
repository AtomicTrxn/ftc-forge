package simrunner;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

class SimConfigTest {
    @TempDir Path team;

    @Test void parsesServoShaftSpecAndRequiresEveryField() throws Exception {
        String fields = "\"stall_torque_nm\":1.2,\"no_load_speed_rad_s\":5,"
            + "\"travel_rad\":3.14,\"position_gain_per_s\":10,"
            + "\"velocity_gain_nm_per_rad_s\":0.5,\"deadband_rad\":0.01";
        Files.writeString(team.resolve("sim.config"), "{\"servoPhysics\":{\"claw\":{" + fields + "}}}");
        var config = SimConfig.load(team);
        assertEquals(1.2, config.servoPhysics.get("claw").stallTorqueNm());
        Files.writeString(team.resolve("sim.config"), "{\"servoPhysics\":{\"claw\":{"
            + fields.replace(",\"deadband_rad\":0.01", "") + "}}}");
        assertThrows(IllegalArgumentException.class, () -> SimConfig.load(team));
    }
    @Test void contactConfigurationUsesUrdfBoundsAndRejectsMalformedMaterialParameters() throws Exception {
        String spec="\"links\":[\"flap\"],\"segments_per_arm\":3,\"flex_mass_fraction\":0.9,"
            +"\"width_m\":0.012,\"arm_length_m\":0.05,\"thickness_m\":0.003,\"stiffness_nm_per_rad\":0.003,"
            +"\"damping_ratio\":0.5,\"max_bend_rad\":1.2,\"friction\":1,\"contact_stiffness_n_per_m\":500,"
            +"\"contact_damping_ns_per_m\":0.3,\"containment_min_xyz_m\":[-0.02,-0.105,0],\"containment_max_xyz_m\":[0.2,0.105,0.1]";
        String json="{\"urdf\":\"robot.urdf\",\"intake\":{\"motor\":\"intake\",\"shaft_sign\":1,\"min_speed_rad_s\":1,"
            +"\"point_xyz_m\":[0.24,0,0.06],\"capture_radius_m\":0.12},\"flexible_intake\":{"+spec+"}}";
        Files.writeString(team.resolve("sim.config"),json);
        var c=SimConfig.load(team).flexibleIntake;
        assertEquals(-.105,c.containmentMin().z,.00001);assertEquals(.105,c.containmentMax().z,.00001);
        c.containmentMin().set(100,100,100);assertEquals(-.02,c.containmentMin().x,.00001);
        for(String bad:new String[]{json.replace("segments_per_arm\":3","segments_per_arm\":3.5"),
                json.replace("flex_mass_fraction\":0.9","flex_mass_fraction\":1"),json.replace("stiffness_nm_per_rad\":0.003","stiffness_nm_per_rad\":-1"),
                json.replace("contact_stiffness_n_per_m\":500","contact_stiffness_n_per_m\":0"),json.replace("[\"flap\"]","[\"flap\",\"flap\"]")}) {
            Files.writeString(team.resolve("sim.config"),bad);assertThrows(IllegalArgumentException.class,()->SimConfig.load(team));
        }
    }
    @Test void validatesDriveAndMotorIntakeConfiguration() throws Exception {
        String json = "{\"drive\":{\"type\":\"differential\",\"left_motor\":\"left\",\"right_motor\":\"right\","
            + "\"track_width_m\":0.38,\"wheel_radius_m\":0.045,\"left_shaft_sign\":-1,\"right_shaft_sign\":1},"
            + "\"intake\":{\"motor\":\"intake\",\"shaft_sign\":1,\"min_speed_rad_s\":1,\"point_xyz_m\":[0.24,0,0.06],\"capture_radius_m\":0.12}}";
        Files.writeString(team.resolve("sim.config"), json);
        var config = SimConfig.load(team);
        assertEquals(.38, config.drive.trackWidthM()); assertEquals(.06, config.intake.point().y, .001);
        for (String invalid : new String[]{json.replace("0.38", "0"), json.replace("\"right\"", "\"left\""),
                json.replace("-1", "0"), json.replace("[0.24,0,0.06]", "[0.24]"), json.replace("0.12", "-0.12"),
                json.replace("differential", "unknown")}) {
            Files.writeString(team.resolve("sim.config"), invalid);
            assertThrows(IllegalArgumentException.class, () -> SimConfig.load(team));
        }
    }
}
