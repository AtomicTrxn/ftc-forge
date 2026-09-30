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
