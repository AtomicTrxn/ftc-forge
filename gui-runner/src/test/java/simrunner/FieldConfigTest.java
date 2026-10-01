package simrunner;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.*;
import java.util.Map;
import static org.junit.jupiter.api.Assertions.*;

class FieldConfigTest {
    @TempDir Path temp;
    @Test void defaultsToGenericFieldOnlyIndependentlyOfRobotModel()throws Exception {
        Files.writeString(temp.resolve("sim.config"),"{\"urdf\":\"robot.urdf\"}");
        var config=SimConfig.load(temp);
        assertEquals("generic",config.field.source());assertFalse(config.field.hasPieces());assertEquals("robot.urdf",config.urdf);
        assertFalse(config.field.withMode("game-pieces").usesTorus());
    }
    @Test void explicitlySelectedMissingFieldFailsAndUnknownModesUnitsFail()throws Exception {
        assertThrows(IllegalArgumentException.class,()->new FieldPackage(temp.resolve("missing.json")));
        assertThrows(IllegalArgumentException.class,()->FieldConfig.parse(Map.of("source","imported")));
        assertThrows(IllegalArgumentException.class,()->FieldConfig.parse(Map.of("mode","sometimes")));
        assertThrows(IllegalArgumentException.class,()->FieldConfig.parse(Map.of("friction",-1)));
        assertThrows(IllegalArgumentException.class,()->FieldConfig.parse(Map.of("full_detail","true")));
        var path=temp.resolve("field.json");Files.writeString(path,"{\"version\":1,\"units\":\"mm\"}");
        assertThrows(IllegalArgumentException.class,()->new FieldPackage(path));
    }
    @Test void torusPracticeIsExplicitAndImportUsesActualBalls() {
        var torus=FieldConfig.torusPractice();assertTrue(torus.usesTorus());
        var imported=torus.withSource("field.json");assertEquals("biobuzz",imported.pieceSet());assertFalse(imported.usesTorus());
        assertEquals("field-only",imported.withMode("field-only").mode());
    }
    @Test void sharedFrameStartAndFiniteYawAreParsedWithoutRescaling()throws Exception {
        Files.writeString(temp.resolve("sim.config"),"{\"robot_start_xyz_m\":[1,2,0.1],\"robot_start_yaw_rad\":1.57}");
        var c=SimConfig.load(temp);assertEquals(1,c.robotStart.x);assertEquals(.1,c.robotStart.y,.00001);assertEquals(-2,c.robotStart.z);assertEquals(1.57,c.robotYawRad,.001);
        Files.writeString(temp.resolve("sim.config"),"{\"robot_start_xyz_m\":[1,2,-0.1]}");
        assertThrows(IllegalArgumentException.class,()->SimConfig.load(temp));
    }
}
