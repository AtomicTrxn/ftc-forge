package simcore;

import com.qualcomm.hardware.lynx.LynxModule;
import physics.InterpolatedSensorBuffer;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.locks.LockSupport;

/**
 * A simulated hub: a LynxModule (so team code can call getAll(LynxModule.class) for bulk
 * caching, as the Road Runner quickstart does) whose VoltageSensor reading reflects the
 * real, shared battery-sag voltage HardwareMapBuilder computes each tick (R4), not a constant.
 *
 * It also models what the real hub does with its motors' encoder reads: each read costs bus
 * time ({@code readCostMs}), and the bulk-caching mode decides how often that cost is paid and
 * how stale a value can be:
 * <ul>
 *   <li>OFF: every read is a fresh bus transaction.</li>
 *   <li>AUTO: reads share one snapshot until the same value is read a second time.</li>
 *   <li>MANUAL: reads share one snapshot until {@code clearBulkCache()}; forgetting to clear returns stale data.</li>
 * </ul>
 */
public class SimVoltageSensor extends LynxModule {
    public enum Kind { POSITION, VELOCITY }

    private record Key(SimDcMotorEx motor, Kind kind) { }

    private final InterpolatedSensorBuffer voltageBuffer = new InterpolatedSensorBuffer(1000);
    private final List<SimDcMotorEx> motors = new ArrayList<>();
    private final Set<Key> readSinceSnapshot = new HashSet<>();
    private Map<SimDcMotorEx, SimDcMotorEx.Reading> snapshot;
    private volatile long voltageLatencyMs;
    private volatile double readCostMs;
    private volatile long lastPublishMs = -1;
    private volatile int busTransactions;

    public SimVoltageSensor(String name) { super(name); }

    public synchronized void attach(SimDcMotorEx motor) { motors.add(motor); }

    public void setVoltageLatencyMs(long ms) {
        if (ms < 0) throw new IllegalArgumentException("Voltage latency must be nonnegative");
        voltageLatencyMs = ms;
    }

    public void setReadCostMs(double ms) {
        if (!(ms >= 0)) throw new IllegalArgumentException("Read cost must be nonnegative");
        readCostMs = ms;
    }

    /** Publishes this tick's terminal voltage on the simulator clock. */
    public void publishVoltage(long timeMs, double volts) {
        super.setVoltage(volts);
        voltageBuffer.push(timeMs, volts);
        lastPublishMs = timeMs;
    }

    @Override public double getVoltage() {
        if (voltageLatencyMs == 0 || lastPublishMs < 0) return super.getVoltage();
        double delayed = voltageBuffer.read(lastPublishMs, voltageLatencyMs);
        return Double.isNaN(delayed) ? super.getVoltage() : delayed;
    }

    /** Number of simulated bus round trips so far; lets tests prove how much caching saved. */
    public int busTransactions() { return busTransactions; }

    @Override public synchronized void setBulkCachingMode(BulkCachingMode mode) {
        super.setBulkCachingMode(mode);
        clearBulkCache();
    }

    @Override public synchronized void clearBulkCache() {
        snapshot = null;
        readSinceSnapshot.clear();
    }

    /** Encoder-side read for one of this hub's motors, subject to the caching mode and bus cost. */
    synchronized SimDcMotorEx.Reading read(SimDcMotorEx motor, Kind kind) {
        switch (getBulkCachingMode()) {
            case AUTO: {
                Key key = new Key(motor, kind);
                if (snapshot == null || readSinceSnapshot.contains(key)) {
                    refresh();
                    readSinceSnapshot.clear();
                }
                readSinceSnapshot.add(key);
                return snapshot.get(motor);
            }
            case MANUAL:
                if (snapshot == null) refresh();
                return snapshot.get(motor);
            default:
                transaction();
                return motor.liveReading();
        }
    }

    private void refresh() {
        transaction();
        snapshot = new IdentityHashMap<>();
        for (SimDcMotorEx m : motors) snapshot.put(m, m.liveReading());
    }

    private void transaction() {
        busTransactions++;
        long nanos = (long) (readCostMs * 1_000_000);
        if (nanos > 0) LockSupport.parkNanos(nanos);
    }

    @Override public String getConnectionInfo() { return "Simulated hub \"" + getDeviceName() + "\""; }
}
