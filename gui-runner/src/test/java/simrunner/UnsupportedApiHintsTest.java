package simrunner;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

class UnsupportedApiHintsTest {
    @TempDir Path project;

    @Test void importedNameHandlesStaticWildcardAndNonImports() {
        assertEquals("a.b.C", UnsupportedApiHints.importedName("  import a.b.C;"));
        assertEquals("a.b", UnsupportedApiHints.importedName("import a.b.*;"));
        assertEquals("a.B.m", UnsupportedApiHints.importedName("import static a.B.m;"));
        assertNull(UnsupportedApiHints.importedName("int x = 1;"));
    }

    @Test void adviceCoversVisionAndroidVendoredDriversAndLibraries() {
        assertTrue(UnsupportedApiHints.adviceFor("org.openftc.easyopencv.OpenCvCamera").contains("vision"));
        assertTrue(UnsupportedApiHints.adviceFor("android.content.Context").contains("Android"));
        assertTrue(UnsupportedApiHints.adviceFor("com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice").contains("Pinpoint"));
        assertTrue(UnsupportedApiHints.adviceFor("com.acmerobotics.roadrunner.Pose2d").contains("extraClasspath"));
        assertTrue(UnsupportedApiHints.adviceFor("com.pedropathing.follower.Follower").contains("extraClasspath"));
        assertTrue(UnsupportedApiHints.adviceFor("com.qualcomm.robotcore.hardware.Unknown").contains("SDK_COVERAGE.md"));
        assertNull(UnsupportedApiHints.adviceFor("java.util.List"));
        assertNull(UnsupportedApiHints.adviceFor("org.example.Mine"));
    }

    @Test void notesOnlyIncludeKnownNamespaces() {
        List<String> notes = UnsupportedApiHints.notes(Set.of("org.example.Mine", "org.opencv.core.Mat"));
        assertEquals(1, notes.size());
        assertTrue(notes.get(0).contains("org.opencv.core.Mat"));
    }

    @Test void compileFailureExplainsUnsupportedImports() throws Exception {
        Path source = project.resolve("src/Team.java");
        Files.createDirectories(source.getParent());
        Files.writeString(source, "import org.openftc.easyopencv.OpenCvCamera;\nimport java.util.List;\n"
            + "public class Team { OpenCvCamera camera; }\n");
        IllegalStateException e = assertThrows(IllegalStateException.class,
            () -> TeamCodeCompiler.compile(project, source.getParent(), List.of()));
        assertTrue(e.getMessage().contains("Team code failed to compile"), e.getMessage());
        assertTrue(e.getMessage().contains("Simulator support notes"), e.getMessage());
        assertTrue(e.getMessage().contains("org.openftc.easyopencv.OpenCvCamera: EasyOpenCV"), e.getMessage());
    }

    @Test void ordinaryCompileErrorsGetNoSimulatorNotes() throws Exception {
        Path source = project.resolve("src/Plain.java");
        Files.createDirectories(source.getParent());
        Files.writeString(source, "public class Plain { int x = \"text\"; }\n");
        IllegalStateException e = assertThrows(IllegalStateException.class,
            () -> TeamCodeCompiler.compile(project, source.getParent(), List.of()));
        assertFalse(e.getMessage().contains("Simulator support notes"), e.getMessage());
    }
}
