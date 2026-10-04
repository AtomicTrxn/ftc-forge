package simrunner;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/**
 * Turns "package does not exist" compile errors for known-unsupported FTC SDK / vendor APIs into
 * actionable notes. The simulator stubs only part of the SDK (see core-sdk-mock/SDK_COVERAGE.md);
 * without these notes a team sees an opaque javac error for code that is fine on a robot.
 */
final class UnsupportedApiHints {
    private record Rule(String prefix, String advice) { }

    private static final List<Rule> RULES = List.of(
        new Rule("org.openftc.easyopencv", "EasyOpenCV and camera pipelines are not simulated; vision is deferred. "
            + "Guard camera code behind a flag or remove it for simulator runs."),
        new Rule("org.opencv", "OpenCV is not simulated; vision is deferred. Guard or remove camera code for simulator runs."),
        new Rule("org.firstinspires.ftc.vision.opencv", "OpenCV vision processors are not simulated; "
            + "only VisionPortal and AprilTagProcessor exist as inert stubs."),
        new Rule("org.firstinspires.ftc.robotcore.external.tfod", "TensorFlow object detection is not simulated; vision is deferred."),
        new Rule("com.qualcomm.hardware.limelightvision", "Limelight is not simulated; vision is deferred."),
        new Rule("android.", "Android APIs are unavailable on the desktop JVM. Move Android-only code (Context, "
            + "Looper, Log, Toast) behind an interface or remove it for simulator runs."),
        new Rule("androidx.", "AndroidX is unavailable on the desktop JVM."),
        new Rule("com.qualcomm.ftccommon", "ftccommon (Driver Station / robot controller internals) is not simulated."),
        new Rule("com.qualcomm.robotcore.hardware.I2cDeviceSynch", "Raw I2C device classes are not simulated, so a vendored "
            + "I2C driver (e.g. a copied GoBildaPinpointDriver.java) will not compile. Delete the copy and use the built-in "
            + "com.qualcomm.hardware.gobilda.GoBildaPinpointDriver or com.qualcomm.hardware.sparkfun.SparkFunOTOS."),
        new Rule("com.qualcomm.hardware.lynx.LynxI2cDeviceSynch", "Raw Lynx I2C access is not simulated; use the built-in "
            + "Pinpoint/OTOS classes instead of a vendored driver."),
        new Rule("com.qualcomm.robotcore.hardware.configuration", "Hardware-configuration annotations are not simulated; "
            + "devices come from the robot configuration XML."),
        new Rule("com.acmerobotics.roadrunner", "Road Runner is a real library, not part of the simulator. Add its jars "
            + "(road-runner core/ftc and Kotlin stdlib) to extraClasspath in sim.config."),
        new Rule("com.pedropathing", "Pedro Pathing is a real library, not part of the simulator. Add its jars and Kotlin/other "
            + "dependencies to extraClasspath in sim.config."),
        new Rule("com.acmerobotics.dashboard", "This FtcDashboard class is not in the simulator's dashboard shim "
            + "(FtcDashboard, @Config, TelemetryPacket, Canvas, MultipleTelemetry)."),
        new Rule("com.qualcomm.", "This FTC SDK class is not in the simulator's stub surface. See core-sdk-mock/SDK_COVERAGE.md "
            + "for what exists, and open an issue for the missing API."),
        new Rule("org.firstinspires.ftc.", "This FTC SDK class is not in the simulator's stub surface. See "
            + "core-sdk-mock/SDK_COVERAGE.md for what exists, and open an issue for the missing API."));

    private UnsupportedApiHints() { }

    /** Advice for the first matching rule, or null if the import is not in a known SDK namespace. */
    static String adviceFor(String importedName) {
        for (Rule rule : RULES) {
            if (importedName.startsWith(rule.prefix)) return rule.advice;
        }
        return null;
    }

    /** One note per unresolved import that matches a rule; empty when there is nothing to say. */
    static List<String> notes(Set<String> unresolvedImports) {
        List<String> notes = new ArrayList<>();
        Set<String> seen = new LinkedHashSet<>();
        for (String name : unresolvedImports) {
            String advice = adviceFor(name);
            if (advice != null && seen.add(name)) notes.add("  " + name + ": " + advice);
        }
        return notes;
    }

    /** Extracts the imported name from a source line such as {@code import a.b.C;}; null otherwise. */
    static String importedName(String sourceLine) {
        String line = sourceLine.strip();
        if (!line.startsWith("import ") || !line.endsWith(";")) return null;
        String name = line.substring("import ".length(), line.length() - 1).strip();
        if (name.startsWith("static ")) name = name.substring("static ".length()).strip();
        return name.endsWith(".*") ? name.substring(0, name.length() - 2) : name;
    }
}
