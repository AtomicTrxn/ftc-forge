package simcore;

import org.junit.jupiter.api.Test;
import physics.MotorSpec;

import static org.junit.jupiter.api.Assertions.*;

class EncoderLatencyTest {
    private static final double RAD_S = 10;

    /** Constant-speed shaft ticked every {@code tickMs}; returns the lag (ms) between true and read encoder at the last tick. */
    private static double lagMs(long latencyMs, long tickMs) {
        SimDcMotorEx motor = new SimDcMotorEx("m", new MotorSpec("t", 1, 2, 9, 30, 12, 1000));
        motor.setEncoderLatencyMs(latencyMs);
        motor.useExternalShaft();
        double rad = 0;
        long t = 0;
        for (int i = 0; i < 50; i++) {
            motor.syncExternalShaft(rad, RAD_S);
            motor.integrate(12, tickMs / 1000.0, t);
            rad += RAD_S * tickMs / 1000.0;
            t += tickMs;
        }
        // Last integrate ran at time t - tickMs with shaft angle rad - RAD_S*tick.
        double trueTicks = (rad - RAD_S * tickMs / 1000.0) * 1000 / (2 * Math.PI);
        double ticksPerMs = RAD_S * 1000 / (2 * Math.PI) / 1000.0;
        return (trueTicks - motor.getCurrentPosition()) / ticksPerMs;
    }

    @Test void configuredLatencyIsHonouredWhenTheTickIsCoarserThanTheLatency() {
        assertEquals(8, lagMs(8, 20), 1.0);
        assertEquals(30, lagMs(30, 20), 1.0);
        assertEquals(0, lagMs(0, 20), 1.0);
    }
}
