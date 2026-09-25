package simcore;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.jupiter.api.Test;

import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

class RobotUrdfTest {
    private static Path sampleDir() { return Path.of("..", "gui-runner", "sample-teamcode"); }

    @Test void workedExampleResolvesAndMassOverridePreservesDistribution() throws Exception {
        RobotUrdf urdf = RobotUrdf.parse(sampleDir().resolve("robot.urdf"));
        HardwareMap map = HardwareMapBuilder.build(
            RobotConfigXml.parse(sampleDir().resolve("robot_config.xml").toFile()),
            PresetRobotConfig.load(sampleDir().resolve("preset_motors.json")));
        urdf.validateHardwareMap(map);
        assertEquals(7, urdf.links.size());
        assertEquals(6, urdf.joints.size());
        assertEquals("prismatic", urdf.joints.get("slide").type());
        assertEquals("revolute", urdf.joints.get("arm").type());
        assertEquals("continuous", urdf.joints.get("wheel_lf").type());
        double sourceMass = urdf.totalMassKg();
        RobotUrdf scaled = urdf.withTotalMassKg(11.0);
        assertEquals(11.0, scaled.totalMassKg(), 1e-9);
        assertEquals(urdf.links.get("arm_link").massKg() * 11.0 / sourceMass,
            scaled.links.get("arm_link").massKg(), 1e-9);
        assertEquals(urdf.links.get("arm_link").inertia().ixx() * 11.0 / sourceMass,
            scaled.links.get("arm_link").inertia().ixx(), 1e-9);
    }

    @Test void twoMotorSlideResolvesBothActuatorsAndRejectsMissingOne() throws Exception {
        String xml = Files.readString(sampleDir().resolve("robot.urdf"));
        xml = xml.replace("</actuator></transmission>\n</robot>",
            "</actuator><actuator name=\"slide_right\"><mechanicalReduction>52.6</mechanicalReduction></actuator></transmission>\n</robot>");
        Path temp = Files.createTempFile("multi-slide", ".urdf");
        try {
            Files.writeString(temp, xml);
            RobotUrdf urdf = RobotUrdf.parse(temp);
            assertEquals(2, urdf.transmissions.get("slide_tx").actuators().size());
            HardwareMap map = HardwareMapBuilder.build(
                RobotConfigXml.parse(sampleDir().resolve("robot_config.xml").toFile()),
                PresetRobotConfig.load(sampleDir().resolve("preset_motors.json")));
            IllegalArgumentException missing = assertThrows(IllegalArgumentException.class,
                () -> urdf.validateHardwareMap(map));
            assertTrue(missing.getMessage().contains("slide_right"));
            String configXml = Files.readString(sampleDir().resolve("robot_config.xml"));
            configXml = configXml.replace("<goBILDA5202SeriesMotor name=\"slide_motor\" port=\"1\" bus=\"1\"/>",
                "<goBILDA5202SeriesMotor name=\"slide_motor\" port=\"1\" bus=\"1\"/>"
                    + "<goBILDA5202SeriesMotor name=\"slide_right\" port=\"2\" bus=\"1\"/>");
            Path pairedXml = Files.createTempFile("multi-slide", ".xml");
            try {
                Files.writeString(pairedXml, configXml);
                HardwareMap pairedMap = HardwareMapBuilder.build(RobotConfigXml.parse(pairedXml.toFile()),
                    PresetRobotConfig.load(sampleDir().resolve("preset_motors.json")));
                assertDoesNotThrow(() -> urdf.validateHardwareMap(pairedMap));
                assertNotNull(pairedMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "slide_right"));
            } finally {
                Files.deleteIfExists(pairedXml);
            }
        } finally {
            Files.deleteIfExists(temp);
        }
    }
}
