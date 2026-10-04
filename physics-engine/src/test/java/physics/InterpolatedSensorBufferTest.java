package physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class InterpolatedSensorBufferTest {
    @Test void interpolatesBetweenTicksSoShortLatenciesAreHonoured() {
        InterpolatedSensorBuffer buf = new InterpolatedSensorBuffer(1000);
        buf.push(0, 0);
        buf.push(20, 200);
        assertEquals(120, buf.read(20, 8), 1e-9);     // 8 ms behind a 20 ms tick
        assertEquals(200, buf.read(20, 0), 1e-9);
        assertEquals(0, buf.read(20, 20), 1e-9);
    }

    @Test void noDataBeforeTheFirstSampleReturnsNaNAndNewestIsHeld() {
        InterpolatedSensorBuffer buf = new InterpolatedSensorBuffer(1000);
        assertTrue(Double.isNaN(buf.read(0, 0)));
        buf.push(100, 5);
        assertTrue(Double.isNaN(buf.read(100, 50)));  // target t=50 predates the first sample
        assertEquals(5, buf.read(500, 0), 1e-9);
    }

    @Test void oldSamplesAreEvictedButTheWindowEdgeStillInterpolates() {
        InterpolatedSensorBuffer buf = new InterpolatedSensorBuffer(50);
        for (int t = 0; t <= 200; t += 20) buf.push(t, t);
        assertEquals(170, buf.read(200, 30), 1e-9);
        assertTrue(Double.isNaN(buf.read(200, 150)));
    }

    @Test void aRestartedClockDiscardsTheOldHistory() {
        InterpolatedSensorBuffer buf = new InterpolatedSensorBuffer(100);
        buf.push(1000, 1);
        buf.push(0, 7);
        assertEquals(7, buf.read(0, 0), 1e-9);
        assertTrue(Double.isNaN(buf.read(0, 10)));
    }
}
