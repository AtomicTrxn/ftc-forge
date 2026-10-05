package simcore;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.apriltag.*;
import static org.junit.jupiter.api.Assertions.*;
class SceneHardwareTest {
    @TempDir Path tmp;
    @Test void exportedXmlCameraAndSensorsBindToActualTeamCodeTypes()throws Exception {
        Path xml=tmp.resolve("hardware.xml"),preset=tmp.resolve("motors.json");Files.writeString(xml,"<Robot><Webcam name='camera'/><LynxModule name='hub'><DistanceSensor name='range'/><ColorSensor name='color'/><TouchSensor name='switch'/></LynxModule></Robot>");Files.writeString(preset,"{\"name\":\"test\",\"motors\":{}}");
        var hardware=HardwareMapBuilder.build(RobotConfigXml.parse(xml.toFile()),PresetRobotConfig.load(preset));assertNotNull(hardware.get(SimDistanceSensor.class,"range"));assertNotNull(hardware.get(SimColorSensor.class,"color"));assertNotNull(hardware.get(SimTouchSensor.class,"switch"));var camera=hardware.get(WebcamName.class,"camera");
        var processor=new AprilTagProcessor.Builder().build();var portal=new VisionPortal.Builder().setCamera(camera).addProcessor(processor).build();assertEquals(VisionPortal.CameraState.CAMERA_DEVICE_CLOSED,portal.getCameraState());assertTrue(processor.getDetections().isEmpty());portal.close();
    }
}
