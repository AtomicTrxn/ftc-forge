package simrunner;

import javax.tools.*;
import java.io.IOException;
import java.io.File;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.*;
import java.util.*;
import java.util.stream.Collectors;

/**
 * Runtime compilation of team source via javax.tools.JavaCompiler (per R3's classloading
 * design). Requires a full JDK (jdk.compiler module), not a bare JRE -- R3/R2's packaging
 * note, carried forward here as a hard runtime requirement, not just documentation.
 */
public class TeamCodeCompiler {

    public static Path compile(Path projectDir, Path sourceRoot, List<String> extraClasspath) throws IOException {
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

        List<Path> dependencies = resolveExtraClasspath(projectDir, extraClasspath);
        List<String> options = new ArrayList<>(List.of(
            "-d", outputDir.toString(),
            "-cp", System.getProperty("java.class.path") + (dependencies.isEmpty() ? "" :
                File.pathSeparator + dependencies.stream().map(Path::toString).collect(Collectors.joining(File.pathSeparator)))
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

    public static URLClassLoader newClassLoader(Path projectDir, Path classesDir, List<String> extraClasspath)
        throws IOException {
        List<URL> urls = new ArrayList<>();
        urls.add(classesDir.toUri().toURL());
        for (Path dependency : resolveExtraClasspath(projectDir, extraClasspath)) {
            urls.add(dependency.toUri().toURL());
        }
        return new URLClassLoader(urls.toArray(URL[]::new), TeamCodeCompiler.class.getClassLoader());
    }

    private static List<Path> resolveExtraClasspath(Path projectDir, List<String> extraClasspath) throws IOException {
        List<Path> resolved = new ArrayList<>();
        for (String entry : extraClasspath) {
            Path path = projectDir.resolve(entry).toAbsolutePath().normalize();
            if (!Files.exists(path)) throw new IOException("extraClasspath entry does not exist: " + path);
            resolved.add(path);
        }
        return resolved;
    }

    private static class StringWriterDiagnostics implements DiagnosticListener<JavaFileObject> {
        final StringBuilder messages = new StringBuilder();
        @Override public void report(Diagnostic<? extends JavaFileObject> diagnostic) {
            messages.append(diagnostic.toString()).append('\n');
        }
    }
}
