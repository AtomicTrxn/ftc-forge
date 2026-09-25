package physics;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class SensorRingBufferTest {

    @Test
    void readReturnsNull_beforeAnySampleExists() {
        SensorRingBuffer<Double> buf = new SensorRingBuffer<>(1000);
        assertNull(buf.read(0, 10));
    }

    @Test
    void readReturnsDelayedHistoricalValue_notTheLatestOne() {
        SensorRingBuffer<Integer> buf = new SensorRingBuffer<>(1000);
        buf.push(0, 100);
        buf.push(10, 200);
        buf.push(20, 300);
        buf.push(30, 400);

        // At sim time 30 with 10ms latency, should see the value from t=20, not t=30.
        assertEquals(300, buf.read(30, 10));
        // With 25ms latency, should see the value from t=0 (last sample at or before t=5).
        assertEquals(100, buf.read(30, 25));
    }

    @Test
    void oldSamplesBeyondMaxAge_areEvicted() {
        SensorRingBuffer<Integer> buf = new SensorRingBuffer<>(50);
        buf.push(0, 1);
        buf.push(1000, 2); // far beyond maxAgeMs -- should evict the t=0 sample
        assertEquals(2, buf.read(1000, 0));
        assertNull(buf.read(1000, 999)); // t=0 sample no longer retained
    }
}
