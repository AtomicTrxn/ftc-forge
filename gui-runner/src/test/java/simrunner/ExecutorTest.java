package simrunner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.jupiter.api.Test;
import simcore.ConsoleTelemetry;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import static org.junit.jupiter.api.Assertions.*;

class ExecutorTest {
    public static class LifecycleOpMode extends OpMode {
        static final List<String> events = new CopyOnWriteArrayList<>();
        @Override public void init() { events.add("init"); }
        @Override public void init_loop() { events.add("init_loop"); }
        @Override public void start() { events.add("start"); }
        @Override public void loop() {
            events.add("loop");
            if (events.stream().filter("loop"::equals).count() >= 3) requestOpModeStop();
        }
        @Override public void stop() { events.add("stop"); }
    }

    @Test void iterativeLifecycleRunsInOrderAndStops() throws Exception {
        LifecycleOpMode.events.clear();
        var mode = new OpModeDiscovery.DiscoveredOpMode(LifecycleOpMode.class.getName(),
            "Lifecycle", false, LifecycleOpMode.class);
        Executor.RunResult result = Executor.runOpMode(mode, new HardwareMap(), new ConsoleTelemetry(),
            new Gamepad(), new Gamepad(), 2000);
        assertTrue(result.completedNormally);
        assertFalse(result.watchdogTriggered);
        assertNull(result.failure);
        assertEquals("init", LifecycleOpMode.events.get(0));
        assertTrue(LifecycleOpMode.events.indexOf("init_loop") < LifecycleOpMode.events.indexOf("start"));
        assertTrue(LifecycleOpMode.events.indexOf("start") < LifecycleOpMode.events.indexOf("loop"));
        assertEquals("stop", LifecycleOpMode.events.get(LifecycleOpMode.events.size() - 1));
        assertEquals(3, LifecycleOpMode.events.stream().filter("loop"::equals).count());
    }
}
