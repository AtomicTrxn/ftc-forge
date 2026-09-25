package simrunner;

import javax.tools.*;
import java.io.IOException;
import java.nio.file.*;
import java.util.*;
import java.util.stream.Collectors;

/**
 * Runtime compilation of team source via javax.tools.JavaCompiler (per R3's classloading
 * design). Requires a full JDK (jdk.compiler module), not a bare JRE -- R3/R2's packaging
 * note, carried forward here as a hard runtime requirement, not just documentation.
 */
public class TeamCodeCompiler {

    public static Path compile(Path sourceRoot, List<String> extraClasspath) throws IOException {
        JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
        if (compiler == null) {
            throw new IllegalStateException(
                "No system Java compiler available -- this requires running on a full JDK "
                    + "(jdk.compiler module), not a bare JRE. See R3's packaging note.");
        }

        Path outputDir = Files.createTempDirectory("sim-teamcode-classes-");

        List<Path> sourceFiles;
        try (var walk = Files.walk(sourceRoot)) {
            sourceFiles = walk.filter(p -> p.toString().endsWith(".java")).collect(Collectors.toList());
        }
        if (sourceFiles.isEmpty()) {
            throw new IllegalStateException("No .java files found under " + sourceRoot);
        }

        StandardJavaFileManager fileManager = compiler.getStandardFileManager(null, null, null);
        Iterable<? extends JavaFileObject> units = fileManager.getJavaFileObjectsFromPaths(sourceFiles);

        List<String> options = new ArrayList<>(List.of(
            "-d", outputDir.toString(),
            "-cp", System.getProperty("java.class.path") + (extraClasspath.isEmpty() ? "" : (":" + String.join(":", extraClasspath)))
        ));

        StringWriterDiagnostics diagnostics = new StringWriterDiagnostics();
        JavaCompiler.CompilationTask task = compiler.getTask(
            null, fileManager, diagnostics, options, null, units);

        boolean success = task.call();
        fileManager.close();

        if (!success) {
            throw new IllegalStateException("Team code failed to compile:\n" + diagnostics.messages);
        }
        return outputDir;
    }

    private static class StringWriterDiagnostics implements DiagnosticListener<JavaFileObject> {
        final StringBuilder messages = new StringBuilder();
        @Override public void report(Diagnostic<? extends JavaFileObject> diagnostic) {
            messages.append(diagnostic.toString()).append('\n');
        }
    }
}
