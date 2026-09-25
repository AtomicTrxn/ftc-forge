package com.qualcomm.robotcore.util;

/**
 * Mock of com.qualcomm.robotcore.util.ElapsedTime.
 * Per R3's simulated-time decision (Option A for v1): runs in real time on the wall clock --
 * no pause/fast-forward in v1 -- so this reads System.nanoTime() directly rather than a
 * separate virtual clock.
 */
public class ElapsedTime {
    private long startNanos;

    public ElapsedTime() {
        reset();
    }

    public void reset() {
        startNanos = System.nanoTime();
    }

    public double seconds() {
        return (System.nanoTime() - startNanos) / 1_000_000_000.0;
    }

    public double milliseconds() {
        return (System.nanoTime() - startNanos) / 1_000_000.0;
    }

    public long nanoseconds() {
        return System.nanoTime() - startNanos;
    }
}
