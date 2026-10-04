package simrunner;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.jupiter.api.Test;
import simcore.HardwareMapBuilder;
import simcore.PresetRobotConfig;
import simcore.RobotConfigXml;

import java.io.PrintStream;
import java.io.ByteArrayOutputStream;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Runs the Road Runner- and Pedro-style fixtures through the real compile/discover/execute path. */
class CompatFixturesTest {
    private static final Path PROJECT = Path.of("compat-fixtures");

    /** Runs one fixture headless and returns everything it printed. */
    private static String run(String opModeName) throws Exception {
        SimConfig config = SimConfig.load(PROJECT);
        Path classes = TeamCodeCompiler.compile(PROJECT, PROJECT.resolve(config.sourceRoot), config.extraClasspath);
        try (var loader = TeamCodeCompiler.newClassLoader(PROJECT, classes, config.extraClasspath)) {
            var target = OpModeDiscovery.discover(classes, loader).stream()
                .filter(d -> d.className.endsWith("." + opModeName)).findFirst().orElseThrow();
            HardwareMap map = HardwareMapBuilder.build(RobotConfigXml.parse(PROJECT.resolve(config.robotConfig).toFile()),
                PresetRobotConfig.load(PROJECT.resolve(config.presetMotors)));
            List<String> lines = new ArrayList<>();
            Telemetry telemetry = new simcore.ConsoleTelemetry() {
                @Override public boolean update() { lines.add("update"); return super.update(); }
            };
            PrintStream original = System.out;
            ByteArrayOutputStream captured = new ByteArrayOutputStream();
            System.setOut(new PrintStream(captured));
            try {
                Executor.RunResult result = Executor.runOpMode(target, map, telemetry, new Gamepad(), new Gamepad(), 10_000);
                assertTrue(result.completedNormally, opModeName + " did not complete");
                assertFalse(result.watchdogTriggered);
                assertNull(result.failure);
            } finally {
                System.setOut(original);
            }
            return captured.toString();
        }
    }

    @Test void roadRunnerStyleOpModeRunsWithBulkCachingVoltageAndDashboardOverlay() throws Exception {
        String output = run("RoadRunnerStyleOpMode");
        assertTrue(output.contains("RR_STYLE : done overlayCalls=4 ticks="), output);
        assertTrue(output.contains("[DASHBOARD] leftFrontTicks = "), output);
        assertFalse(output.contains("overlayCalls=4 ticks=0"), "left front encoder should advance under power: " + output);
        assertTrue(output.contains("voltage : 11."), "battery sag should be visible through the hub voltage sensor: " + output);
    }

    @Test void pedroStyleOpModeConfiguresPinpointAndReadsOtos() throws Exception {
        String output = run("PedroStyleOpMode");
        // setPosition(9 in, 8 in) holds headless (no physics world feeds the pose).
        assertTrue(output.contains("PEDRO_STYLE : done loops=5 startX_in=9.0"), output);
        assertTrue(output.contains("otos x (in) : 0.0"), output);
    }
}
