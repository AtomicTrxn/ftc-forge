package simrunner;

/**
 * Runaway-OpMode isolation per R3's design: Thread.stop() throws UnsupportedOperationException
 * unconditionally on JDK 20+ (R2's finding), so a wedged OpMode thread cannot be force-killed.
 * This watchdog instead detects the wedge and marks the session dead so the executor moves on
 * and refuses further interaction with it -- the stuck daemon thread leaks until the JVM
 * process itself restarts, a documented v1 limitation, not a bug.
 */
public class Watchdog {
    private final Thread targetThread;
    private final long timeoutMillis;
    private volatile boolean sessionDead = false;
    private volatile boolean finished = false;

    public Watchdog(Thread targetThread, long timeoutMillis) {
        this.targetThread = targetThread;
        this.timeoutMillis = timeoutMillis;
    }

    public void start() {
        Thread watcher = new Thread(() -> {
            try {
                targetThread.join(timeoutMillis);
            } catch (InterruptedException ignored) {
            }
            if (targetThread.isAlive()) {
                sessionDead = true;
                System.out.println("[WATCHDOG] OpMode thread \"" + targetThread.getName()
                    + "\" did not finish within " + timeoutMillis + "ms -- marking session dead."
                    + " Thread.stop() is unavailable on modern JDKs, so the thread is isolated"
                    + " (left running as an orphaned daemon) rather than force-killed.");
            } else {
                finished = true;
            }
        }, "sim-watchdog");
        watcher.setDaemon(true);
        watcher.start();
        try {
            watcher.join();
        } catch (InterruptedException ignored) {
        }
    }

    public boolean isSessionDead() { return sessionDead; }
    public boolean finishedCleanly() { return finished; }
}
