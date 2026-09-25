package simrunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;
import java.net.URLClassLoader;
import java.nio.file.*;
import java.util.*;
import java.util.stream.Collectors;

/**
 * Discovers runnable OpModes by scanning compiled classes for @TeleOp/@Autonomous -- the
 * real SDK's own decentralized discovery mechanism (per R3), not a manually-named entry
 * point. Loads candidates defensively (Class.forName(name, false, loader), catching
 * LinkageError per class) so one broken class doesn't abort discovery for the whole project
 * (per R3's hardening note).
 */
public class OpModeDiscovery {

    public static class DiscoveredOpMode {
        public final String className;
        public final String displayName;
        public final boolean isAutonomous;
        public final Class<? extends OpMode> opModeClass;

        DiscoveredOpMode(String className, String displayName, boolean isAutonomous, Class<? extends OpMode> opModeClass) {
            this.className = className;
            this.displayName = displayName;
            this.isAutonomous = isAutonomous;
            this.opModeClass = opModeClass;
        }
    }

    public static List<DiscoveredOpMode> discover(Path classesDir, URLClassLoader loader) throws IOException {
        List<String> classNames;
        try (var walk = Files.walk(classesDir)) {
            classNames = walk.filter(p -> p.toString().endsWith(".class"))
                .map(p -> toClassName(classesDir, p))
                .collect(Collectors.toList());
        }

        List<DiscoveredOpMode> found = new ArrayList<>();
        for (String className : classNames) {
            try {
                Class<?> clazz = Class.forName(className, false, loader);

                if (clazz.isAnnotationPresent(Disabled.class)) {
                    continue;
                }
                boolean isTeleOp = clazz.isAnnotationPresent(TeleOp.class);
                boolean isAutonomous = clazz.isAnnotationPresent(Autonomous.class);
                if (!isTeleOp && !isAutonomous) {
                    continue;
                }
                if (!OpMode.class.isAssignableFrom(clazz) || java.lang.reflect.Modifier.isAbstract(clazz.getModifiers())) {
                    System.out.println("[WARN] " + className + " has @TeleOp/@Autonomous but isn't a concrete OpMode subclass -- skipping.");
                    continue;
                }

                String displayName = isTeleOp
                    ? emptyToSimpleName(clazz.getAnnotation(TeleOp.class).name(), clazz)
                    : emptyToSimpleName(clazz.getAnnotation(Autonomous.class).name(), clazz);

                @SuppressWarnings("unchecked")
                Class<? extends OpMode> opModeClass = (Class<? extends OpMode>) clazz;
                found.add(new DiscoveredOpMode(className, displayName, isAutonomous, opModeClass));
            } catch (LinkageError | ClassNotFoundException e) {
                System.out.println("[WARN] Skipping " + className + " during discovery -- failed to load: " + e);
            }
        }
        return found;
    }

    private static String emptyToSimpleName(String name, Class<?> clazz) {
        return (name == null || name.isEmpty()) ? clazz.getSimpleName() : name;
    }

    private static String toClassName(Path root, Path classFile) {
        String rel = root.relativize(classFile).toString();
        return rel.substring(0, rel.length() - ".class".length()).replace('/', '.').replace('\\', '.');
    }
}
