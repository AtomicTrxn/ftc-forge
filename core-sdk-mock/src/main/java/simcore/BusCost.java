package simcore;

import java.util.concurrent.locks.LockSupport;

/** Blocks the calling thread for a simulated bus round trip. Never call while holding a lock the physics thread needs. */
public final class BusCost {
    private BusCost() { }

    public static void block(double ms) {
        long nanos = (long) (ms * 1_000_000);
        if (nanos > 0) LockSupport.parkNanos(nanos);
    }
}
