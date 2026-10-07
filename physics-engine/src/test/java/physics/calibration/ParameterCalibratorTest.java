package physics.calibration;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class ParameterCalibratorTest {
    private static double[] times(int n, int stepMs) {
        double[] t = new double[n];
        for (int i = 0; i < n; i++) t[i] = i * (double) stepMs;
        return t;
    }

    @Test void alignRecoversLagScaleAndDriftExactlyFromNoiseFreeData() {
        double[] t = times(600, 10);
        double[] ref = new double[t.length], meas = new double[t.length];
        for (int i = 0; i < t.length; i++) ref[i] = Math.sin(t[i] / 1000.0 * 1.3) + 0.4 * Math.sin(t[i] / 1000.0 * 0.5);
        for (int i = 0; i < t.length; i++) {
            double shifted = ParameterCalibrator.interpolate(t, ref, t[i] - 70);
            meas[i] = 0.5 + 1.04 * shifted + 0.003 * t[i] / 1000.0;
        }
        var a = ParameterCalibrator.align(t, List.of(ref), List.of(meas), 150, true);
        assertEquals(70, a.lagMs(), 1);
        assertEquals(0.04, a.scaleError(), 1e-3);
        assertEquals(0.003, a.driftPerS(), 1e-4);
        assertEquals(0, a.noiseStd(), 1e-3);
    }

    @Test void alignRefusesAReferenceThatNeverMovesOrIsLinearInTime() {
        double[] t = times(300, 10);
        double[] flat = new double[t.length];
        assertThrows(IllegalArgumentException.class, () -> ParameterCalibrator.align(t, List.of(flat), List.of(flat), 50, false));
        double[] ramp = new double[t.length];
        for (int i = 0; i < t.length; i++) ramp[i] = t[i] / 1000.0;
        assertThrows(IllegalArgumentException.class, () -> ParameterCalibrator.align(t, List.of(ramp), List.of(ramp), 50, true));
    }

    @Test void unwrapRemovesTwoPiJumps() {
        double[] wrapped = {3.0, 3.1, -3.1, -3.0};
        double[] out = ParameterCalibrator.unwrap(wrapped);
        assertEquals(3.0 + 0.1 + (2 * Math.PI - 6.2), out[2], 1e-9);
        assertTrue(out[3] > out[0]);
    }

    @Test void leastSquaresSolvesAKnownLinearSystemAndRefusesConstantColumns() {
        double[][] x = new double[50][];
        double[] y = new double[50];
        for (int i = 0; i < 50; i++) { x[i] = new double[]{1, i, Math.sin(i)}; y[i] = 2 + 3 * i - 4 * Math.sin(i); }
        double[] beta = ParameterCalibrator.leastSquares(x, y, "singular");
        assertArrayEquals(new double[]{2, 3, -4}, beta, 1e-8);
        for (int i = 0; i < 50; i++) x[i][2] = 5;
        assertThrows(IllegalArgumentException.class, () -> ParameterCalibrator.leastSquares(x, y, "singular"));
    }

    @Test void interpolateClampsAtTheEnds() {
        double[] x = {0, 10, 20}, y = {0, 100, 300};
        assertEquals(50, ParameterCalibrator.interpolate(x, y, 5), 1e-12);
        assertEquals(200, ParameterCalibrator.interpolate(x, y, 15), 1e-12);
        assertEquals(0, ParameterCalibrator.interpolate(x, y, -5), 1e-12);
        assertEquals(300, ParameterCalibrator.interpolate(x, y, 99), 1e-12);
    }
}
