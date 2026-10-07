package physics;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Latency model for continuous quantities sampled at the simulator tick (encoder position,
 * velocity). {@link SensorRingBuffer} returns the last sample at or before (now - latency), so a
 * latency shorter than the tick is rounded up to a whole tick; this buffer linearly interpolates
 * between the bracketing samples so the configured latency is honored exactly. Same sim-clock,
 * single-threaded design as R4's ring buffer.
 */
public class InterpolatedSensorBuffer {
    private record Sample(long timeMs, double value) { }

    private final Deque<Sample> samples = new ArrayDeque<>();
    private final long maxAgeMs;

    public InterpolatedSensorBuffer(long maxAgeMs) { this.maxAgeMs = maxAgeMs; }

    public void push(long simTimeMs, double value) {
        // A clock that restarts (a new run reusing the device) invalidates the old history.
        if (!samples.isEmpty() && simTimeMs < samples.peekLast().timeMs) samples.clear();
        samples.addLast(new Sample(simTimeMs, value));
        // Keep one sample older than the window so interpolation at the window edge has a lower bracket.
        while (samples.size() > 2) {
            Sample second = samples.stream().skip(1).findFirst().orElseThrow();
            if (second.timeMs < simTimeMs - maxAgeMs) samples.pollFirst(); else break;
        }
    }

    /**
     * Value at (simTimeMs - latencyMs). Before the first sample it returns NaN (no data yet, like the
     * ring buffer's null); after the newest sample it holds that sample.
     */
    public double read(long simTimeMs, long latencyMs) {
        long target = simTimeMs - latencyMs;
        Sample previous = null;
        for (Sample s : samples) {
            if (s.timeMs > target) {
                if (previous == null) return Double.NaN;
                double span = s.timeMs - previous.timeMs;
                return previous.value + (s.value - previous.value) * (target - previous.timeMs) / span;
            }
            previous = s;
        }
        return previous == null ? Double.NaN : previous.value;
    }
}
