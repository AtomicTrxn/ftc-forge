package simcore;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class OdometryErrorModelTest {
    private static OdometryParams params(long latencyMs, double linear, double headingScale, double drift, double posNoise, double headNoise, long seed) {
        return new OdometryParams(latencyMs, linear, headingScale, drift, posNoise, headNoise, seed);
    }

    /** Robot driving +x at 1 m/s, sampled every 10 ms for {@code ms}. */
    private static GoBildaPinpointDriver drive(OdometryParams p, String name, long ms) {
        GoBildaPinpointDriver pinpoint = new GoBildaPinpointDriver(name);
        pinpoint.configureOdometry(p, name);
        for (long t = 0; t <= ms; t += 10) pinpoint.onChassisPose(t, t / 1000.0, 0, 0, 1, 0, 0);
        pinpoint.update();
        return pinpoint;
    }

    @Test void idealParametersReproduceThePosePerfectly() {
        GoBildaPinpointDriver p = drive(OdometryParams.IDEAL, "pp", 500);
        assertEquals(500, p.getPosX(DistanceUnit.MM), 1e-6);
        assertEquals(1000, p.getVelX(DistanceUnit.MM), 1e-6);
    }

    @Test void latencyReportsThePoseFromEarlierInterpolatedBetweenTicks() {
        GoBildaPinpointDriver p = drive(params(25, 0, 0, 0, 0, 0, 0), "pp", 500);
        assertEquals(475, p.getPosX(DistanceUnit.MM), 1e-6);   // 25 ms * 1 m/s behind, between the 10 ms samples
    }

    @Test void linearAndHeadingScaleErrorsScaleTheReportedMotion() {
        GoBildaPinpointDriver p = new GoBildaPinpointDriver("pp");
        p.configureOdometry(params(0, 0.02, -0.05, 0, 0, 0, 0), "pp");
        p.onChassisPose(0, 0, 0, 0, 0, 0, 0);
        p.onChassisPose(100, 1.0, 0, Math.toRadians(90), 0, 0, 0);
        p.update();
        assertEquals(1020, p.getPosX(DistanceUnit.MM), 1e-6);
        assertEquals(85.5, p.getHeading(AngleUnit.DEGREES), 1e-9);
    }

    @Test void headingDriftAccumulatesWithSimulatedTime() {
        GoBildaPinpointDriver p = new GoBildaPinpointDriver("pp");
        p.configureOdometry(params(0, 0, 0, Math.toRadians(0.5), 0, 0, 0), "pp");
        p.onChassisPose(0, 0, 0, 0, 0, 0, 0);
        p.onChassisPose(10_000, 0, 0, 0, 0, 0, 0);
        p.update();
        assertEquals(5.0, p.getHeading(AngleUnit.DEGREES), 1e-9);
        p.resetPosAndIMU();
        assertEquals(0.0, p.getHeading(AngleUnit.DEGREES), 1e-9);
    }

    @Test void driftAndLatencyAreSkippedWhenTheCallerHasNoClock() {
        GoBildaPinpointDriver p = new GoBildaPinpointDriver("pp");
        p.configureOdometry(params(50, 0, 0, 0.05, 0, 0, 0), "pp");
        p.onChassisPose(0, 0, 0, 0, 0, 0);
        p.onChassisPose(1, 0, 0, 0, 0, 0);
        p.update();
        assertEquals(1000, p.getPosX(DistanceUnit.MM), 1e-6);
        assertEquals(0, p.getHeading(AngleUnit.DEGREES), 1e-9);
    }

    @Test void noiseIsRepeatableForASeedAndDiffersByDeviceNameAndHasTheRequestedSpread() {
        OdometryParams noisy = params(0, 0, 0, 0, 0.002, 0, 42);
        double a1 = drive(noisy, "pinpoint", 500).getPosY(DistanceUnit.MM);
        double a2 = drive(noisy, "pinpoint", 500).getPosY(DistanceUnit.MM);
        double b = drive(noisy, "other", 500).getPosY(DistanceUnit.MM);
        assertEquals(a1, a2, 0.0);
        assertNotEquals(a1, b);
        double sumSq = 0;
        int n = 400;
        for (int i = 0; i < n; i++) {
            double y = drive(params(0, 0, 0, 0, 0.002, 0, i), "pp", 20).getPosY(DistanceUnit.MM);
            sumSq += y * y;
        }
        assertEquals(2.0, Math.sqrt(sumSq / n), 0.3, "std of 2 mm per update");
    }

    @Test void otosAppliesTheSameModel() {
        SparkFunOTOS otos = new SparkFunOTOS("otos");
        otos.setLinearUnit(DistanceUnit.METER);
        otos.configureOdometry(params(0, 0.1, 0, 0, 0, 0, 0), "otos");
        otos.onChassisPose(0, 0, 0, 0, 0, 0, 0);
        otos.onChassisPose(10, 2.0, 0, 0, 0, 0, 0);
        assertEquals(2.2, otos.getPosition().x, 1e-9);
    }

    @Test void parametersAreValidated() {
        assertThrows(IllegalArgumentException.class, () -> params(-1, 0, 0, 0, 0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> params(0, 0.9, 0, 0, 0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> params(0, 0, 0, 0, -1, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> params(0, 0, 0, Double.NaN, 0, 0, 0));
    }
}
