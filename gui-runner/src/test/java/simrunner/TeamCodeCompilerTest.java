package simrunner;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import javax.tools.ToolProvider;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class TeamCodeCompilerTest {
    @TempDir Path project;

    @Test void relativeExtraClasspathWorksAtCompileAndRuntime() throws Exception {
        Path libSource = project.resolve("libsrc/Helper.java");
        Path libClasses = project.resolve("libclasses");
        Files.createDirectories(libSource.getParent());
        Files.createDirectories(libClasses);
        Files.writeString(libSource, "public class Helper { public static String value() { return \"linked\"; } }");
        assertEquals(0, ToolProvider.getSystemJavaCompiler().run(null, null, null,
            "-d", libClasses.toString(), libSource.toString()));

        Path teamSource = project.resolve("TeamCode/src/main/java/Team.java");
        Files.createDirectories(teamSource.getParent());
        Files.writeString(teamSource, "public class Team { public String value() { return Helper.value(); } }");
        Path classes = TeamCodeCompiler.compile(project, teamSource.getParent(), List.of("libclasses"));
        try (var loader = TeamCodeCompiler.newClassLoader(project, classes, List.of("libclasses"))) {
            Object team = loader.loadClass("Team").getDeclaredConstructor().newInstance();
            assertEquals("linked", team.getClass().getMethod("value").invoke(team));
        }
    }
}
