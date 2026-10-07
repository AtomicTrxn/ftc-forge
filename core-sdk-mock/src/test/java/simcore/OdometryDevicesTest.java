package simcore;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class OdometryDevicesTest {
    private static final double EPS = 1e-9;

    @Test void pinpointReportsPoseRelativeToStartInRobotFrame() {
        GoBildaPinpointDriver pinpoint = new GoBildaPinpointDriver("pinpoint");
        // Robot starts at world (2, 3) facing +90 degrees, then drives 1 m along its own heading.
        pinpoint.onChassisPose(2, 3, Math.PI / 2, 0, 0, 0);
        pinpoint.onChassisPose(2, 4, Math.PI / 2, 0, 1, 0.5);
        pinpoint.update();
        assertEquals(1000, pinpoint.getPosX(DistanceUnit.MM), 1e-6);
        assertEquals(0, pinpoint.getPosY(DistanceUnit.MM), 1e-6);
        assertEquals(0, pinpoint.getHeading(AngleUnit.RADIANS), EPS);
        assertEquals(1000, pinpoint.getVelX(DistanceUnit.MM), 1e-6);
        assertEquals(0.5, pinpoint.getHeadingVelocity(), EPS);
    }

    @Test void pinpointStrafeLeftIsPositiveY() {
        GoBildaPinpointDriver pinpoint = new GoBildaPinpointDriver("pinpoint");
        pinpoint.onChassisPose(0, 0, 0, 0, 0, 0);
        pinpoint.onChassisPose(0, 0.25, 0, 0, 0, 0);
        pinpoint.update();
        assertEquals(250, pinpoint.getPosY(DistanceUnit.MM), 1e-6);
    }

    @Test void pinpointSetPositionAndResetRebaseTheReportedPose() {
        GoBildaPinpointDriver pinpoint = new GoBildaPinpointDriver("pinpoint");
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 9, 8, AngleUnit.DEGREES, 90));
        assertEquals(9, pinpoint.getPosition().getX(DistanceUnit.INCH), 1e-9);
        pinpoint.onChassisPose(5, 5, 0, 0, 0, 0);
        pinpoint.update();
        assertEquals(9, pinpoint.getPosX(DistanceUnit.INCH), 1e-9);
        assertEquals(90, pinpoint.getPosition().getHeading(AngleUnit.DEGREES), 1e-9);
        // Moving forward 1 m while reported heading is 90 degrees advances reported +y.
        pinpoint.onChassisPose(6, 5, 0, 0, 0, 0);
        pinpoint.update();
        assertEquals(8 + 1000 / 25.4, pinpoint.getPosY(DistanceUnit.INCH), 1e-9);
        pinpoint.resetPosAndIMU();
        assertEquals(0, pinpoint.getPosX(DistanceUnit.MM), 1e-9);
        assertEquals(0, pinpoint.getHeading(AngleUnit.DEGREES), 1e-9);
    }

    @Test void pinpointHeadingWrapsToHalfTurn() {
        GoBildaPinpointDriver pinpoint = new GoBildaPinpointDriver("pinpoint");
        pinpoint.onChassisPose(0, 0, 0, 0, 0, 0);
        pinpoint.onChassisPose(0, 0, Math.toRadians(95), 0, 0, 0);      // real yaw arrives wrapped, one small step at a time
        pinpoint.onChassisPose(0, 0, Math.toRadians(-170), 0, 0, 0);
        pinpoint.update();
        assertEquals(-170, pinpoint.getHeading(AngleUnit.DEGREES), 1e-9);
        assertEquals(190, pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.DEGREES), 1e-9);
    }

    @Test void otosHonoursConfiguredUnitsAndTracking() {
        SparkFunOTOS otos = new SparkFunOTOS("otos");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.onChassisPose(0, 0, 0, 0, 0, 0);
        otos.onChassisPose(0.0254 * 12, 0, Math.toRadians(45), 0, 0, 0);
        SparkFunOTOS.Pose2D pose = otos.getPosition();
        assertEquals(12, pose.x, 1e-9);
        assertEquals(45, pose.h, 1e-9);
        otos.resetTracking();
        assertEquals(0, otos.getPosition().x, 1e-9);
        otos.setPosition(new SparkFunOTOS.Pose2D(3, 4, 10));
        assertEquals(3, otos.getPosition().x, 1e-9);
        assertEquals(10, otos.getPosition().h, 1e-9);
    }

    @Test void hubsAreLynxModulesAndVoltageSensorsWithBatteryVoltage() throws Exception {
        java.io.File xml = java.io.File.createTempFile("robot", ".xml");
        xml.deleteOnExit();
        java.nio.file.Files.writeString(xml.toPath(), "<Robot><LynxUsbDevice name=\"u\"><LynxModule name=\"Control Hub\">"
            + "<goBILDAPinpoint name=\"pp\" port=\"1\"/><SparkFunOTOS name=\"otos\" port=\"2\"/></LynxModule></LynxUsbDevice></Robot>");
        java.io.File json = java.io.File.createTempFile("preset", ".json");
        json.deleteOnExit();
        java.nio.file.Files.writeString(json.toPath(), "{\"name\":\"p\",\"motors\":{}}");
        HardwareMap map = HardwareMapBuilder.build(RobotConfigXml.parse(xml), PresetRobotConfig.load(json.toPath()));
        assertEquals(1, map.getAll(LynxModule.class).size());
        VoltageSensor voltage = map.voltageSensor.iterator().next();
        assertSame(map.getAll(LynxModule.class).get(0), voltage);
        assertEquals(12.6, voltage.getVoltage(), EPS);
        assertNotNull(map.get(GoBildaPinpointDriver.class, "pp"));
        assertNotNull(map.get(SparkFunOTOS.class, "otos"));
        assertEquals(2, map.getAll(PoseSink.class).size());
    }
}
