package physics;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Deterministic per-sensor latency model, per R4's corrected design (replacing the
 * withdrawn wall-clock DelayQueue): every tick pushes the current true value tagged with
 * sim time; a read returns whatever was true at (now - latencyMs). Single-threaded,
 * no background producer, keyed on the simulator's own clock rather than the wall clock.
 *
 * R4 found this cheap enough to reconsider deferring sensor latency to a fast-follow --
 * Phase 3 ships it now rather than deferring, per that reassessment.
 */
public class SensorRingBuffer<T> {
    private final Deque<Sample<T>> samples = new ArrayDeque<>();
    private final long maxAgeMs;

    private static class Sample<T> {
        final long simTimeMs;
        final T value;
        Sample(long simTimeMs, T value) { this.simTimeMs = simTimeMs; this.value = value; }
    }

    public SensorRingBuffer(long maxAgeMs) {
        this.maxAgeMs = maxAgeMs;
    }

    public void push(long simTimeMs, T value) {
        samples.addLast(new Sample<>(simTimeMs, value));
        while (!samples.isEmpty() && samples.peekFirst().simTimeMs < simTimeMs - maxAgeMs) {
            samples.pollFirst();
        }
    }

    /** Returns the most recent sample at or before (simTimeMs - latencyMs), or null if none exists yet. */
    public T read(long simTimeMs, long latencyMs) {
        long target = simTimeMs - latencyMs;
        T best = null;
        for (Sample<T> s : samples) {
            if (s.simTimeMs <= target) {
                best = s.value;
            } else {
                break;
            }
        }
        return best;
    }
}
