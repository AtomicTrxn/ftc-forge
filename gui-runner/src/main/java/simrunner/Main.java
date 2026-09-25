package simrunner;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import simcore.ConsoleTelemetry;
import simcore.HardwareMapBuilder;
import simcore.PresetRobotConfig;
import simcore.RobotConfigXml;

import java.io.File;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Path;
import java.util.List;

/**
 * Phase 1 headless executor entry point.
 * Usage: java simrunner.Main <projectDir> <opModeClassSimpleNameOrFQCN> [watchdogTimeoutMillis]
 */
public class Main {

    public static void main(String[] args) throws Exception {
        if (args.length < 2) {
            System.err.println("Usage: Main <projectDir> <opModeName> [watchdogTimeoutMillis]");
            System.exit(2);
        }
        Path projectDir = Path.of(args[0]);
        String requestedOpMode = args[1];
        long watchdogTimeoutMillis = args.length >= 3 ? Long.parseLong(args[2]) : 5000;

        System.out.println("=== Sim Executor: loading project at " + projectDir.toAbsolutePath() + " ===");

        SimConfig simConfig = SimConfig.load(projectDir);
        Path sourceRoot = projectDir.resolve(simConfig.sourceRoot);

        System.out.println("[EXECUTOR] Compiling " + sourceRoot + " ...");
        Path classesDir = TeamCodeCompiler.compile(sourceRoot, simConfig.extraClasspath);
        System.out.println("[EXECUTOR] Compiled to " + classesDir);

        // Fresh classloader per reload (R3): static team-code state never survives across runs.
        URL[] urls = { classesDir.toUri().toURL() };
        URLClassLoader teamLoader = new URLClassLoader(urls, Main.class.getClassLoader());

        List<OpModeDiscovery.DiscoveredOpMode> discovered = OpModeDiscovery.discover(classesDir, teamLoader);
        System.out.println("[EXECUTOR] Discovered " + discovered.size() + " OpMode(s) via @TeleOp/@Autonomous scan:");
        for (var d : discovered) {
            System.out.println("    - " + d.displayName + " (" + d.className + ", "
                + (d.isAutonomous ? "Autonomous" : "TeleOp") + ")");
        }

        OpModeDiscovery.DiscoveredOpMode target = discovered.stream()
            .filter(d -> d.className.equals(requestedOpMode) || d.className.endsWith("." + requestedOpMode)
                || d.displayName.equals(requestedOpMode))
            .findFirst()
            .orElseThrow(() -> new IllegalArgumentException("No discovered OpMode matches \"" + requestedOpMode + "\""));

        RobotConfigXml xml = RobotConfigXml.parse(projectDir.resolve(simConfig.robotConfig).toFile());
        PresetRobotConfig preset = PresetRobotConfig.load(projectDir.resolve(simConfig.presetMotors));
        System.out.println("[EXECUTOR] Robot config: " + preset.name + " (" + xml.devices.size() + " devices)");

        HardwareMap hardwareMap = HardwareMapBuilder.build(xml, preset);
        Telemetry telemetry = new ConsoleTelemetry();
        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();

        System.out.println("[EXECUTOR] Running " + target.displayName + " (watchdog timeout " + watchdogTimeoutMillis + "ms)...");
        Executor.RunResult result = Executor.runLinearOpMode(target, hardwareMap, telemetry, gamepad1, gamepad2, watchdogTimeoutMillis);

        if (result.watchdogTriggered) {
            System.out.println("[EXECUTOR] Session marked DEAD by watchdog -- OpMode did not finish in time.");
        } else if (result.completedNormally) {
            System.out.println("[EXECUTOR] OpMode completed normally.");
        }

        System.out.println("=== Final hardware state ===");
        for (var motor : hardwareMap.getAll(com.qualcomm.robotcore.hardware.DcMotorEx.class)) {
            System.out.println("    " + hardwareMap.getNamesOf(motor) + " -> power=" + motor.getPower()
                + " ticks=" + motor.getCurrentPosition());
        }
    }
}
