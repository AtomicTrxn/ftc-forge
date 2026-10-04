package simcore;

import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Keeps core-sdk-mock/sdk-surface.txt honest: the manifest is the single list of what the
 * simulator stubs, so a removed method or an unlisted new stub class fails the build.
 */
class SdkSurfaceTest {
    private static final Set<String> STATUSES = Set.of("simulated", "state-only", "shim", "inert", "support");
    private static final Path MANIFEST = Path.of("sdk-surface.txt");

    private record Entry(String className, String status, Set<String> members) { }

    private static List<Entry> load() throws IOException {
        List<Entry> entries = new ArrayList<>();
        for (String line : Files.readAllLines(MANIFEST)) {
            if (line.isBlank() || line.startsWith("#")) continue;
            String[] parts = line.split("\\|", -1);
            assertEquals(3, parts.length, "Malformed manifest line: " + line);
            Set<String> members = parts[2].isBlank() ? Set.of()
                : new TreeSet<>(Arrays.asList(parts[2].strip().split(",")));
            entries.add(new Entry(parts[0].strip(), parts[1].strip(), members));
        }
        return entries;
    }

    @Test void everyListedClassAndMemberExists() throws Exception {
        List<String> problems = new ArrayList<>();
        Set<String> seen = new HashSet<>();
        for (Entry entry : load()) {
            if (!seen.add(entry.className)) problems.add("duplicate entry " + entry.className);
            if (!STATUSES.contains(entry.status)) problems.add(entry.className + ": unknown status " + entry.status);
            Class<?> type;
            try {
                type = Class.forName(entry.className);
            } catch (ClassNotFoundException e) {
                problems.add("missing class " + entry.className);
                continue;
            }
            Set<String> available = new HashSet<>();
            for (Method m : type.getMethods()) available.add(m.getName());
            for (var f : type.getFields()) available.add(f.getName());
            for (String member : entry.members) {
                if (!available.contains(member)) problems.add(entry.className + " lost public member " + member);
            }
        }
        assertTrue(problems.isEmpty(), String.join("\n", problems));
    }

    @Test void everyStubClassIsListed() throws Exception {
        Path classes = Path.of(Class.forName("com.qualcomm.hardware.lynx.LynxModule")
            .getProtectionDomain().getCodeSource().getLocation().toURI());
        Set<String> listed = load().stream().map(Entry::className).collect(Collectors.toSet());
        Set<String> unlisted = new TreeSet<>();
        try (Stream<Path> walk = Files.walk(classes)) {
            walk.filter(p -> p.toString().endsWith(".class")).forEach(p -> {
                String name = classes.relativize(p).toString().replace('/', '.').replaceAll("\\.class$", "");
                boolean sdkPackage = name.startsWith("com.") || name.startsWith("org.");
                if (sdkPackage && !name.contains("$") && !listed.contains(name)) unlisted.add(name);
            });
        }
        assertTrue(unlisted.isEmpty(), "Stub classes missing from sdk-surface.txt: " + unlisted);
    }

    @Test void deferredVisionAndPinpointAreInTheRoster() throws Exception {
        Map<String, String> status = load().stream().collect(Collectors.toMap(Entry::className, Entry::status));
        assertEquals("inert", status.get("org.firstinspires.ftc.vision.VisionPortal"));
        assertEquals("simulated", status.get("com.qualcomm.hardware.gobilda.GoBildaPinpointDriver"));
        assertEquals("simulated", status.get("com.qualcomm.hardware.sparkfun.SparkFunOTOS"));
        assertEquals("simulated", status.get("com.qualcomm.hardware.lynx.LynxModule"));
    }
}
