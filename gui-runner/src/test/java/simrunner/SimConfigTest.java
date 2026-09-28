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
}
