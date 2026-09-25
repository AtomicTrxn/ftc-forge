package physics;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Validates the kinematics against known test paths, per Phase 2's Definition of Done:
 * "Ideal Mecanum kinematics implemented and correct (verified against a known test path)."
 */
class MecanumKinematicsTest {

    private static final double EPS = 1e-9;
    private final MecanumKinematics k = new MecanumKinematics(0.30, 0.35, 2.0);

    @Test
    void allWheelsForward_producesPureForwardMotionNoStrafeNoRotation() {
        var v = k.forward(1.0, 1.0, 1.0, 1.0);
        assertEquals(2.0, v.vx, EPS);
        assertEquals(0.0, v.vy, EPS);
        assertEquals(0.0, v.omega, EPS);
    }

    @Test
    void strafeLeftPattern_producesPureLateralMotion() {
        // Per this class's documented sign convention: (-1, +1, +1, -1) -> pure strafe left.
        var v = k.forward(-1.0, 1.0, 1.0, -1.0);
        assertEquals(0.0, v.vx, EPS);
        assertEquals(2.0, v.vy, EPS);
        assertEquals(0.0, v.omega, EPS);
    }

    @Test
    void rotateInPlacePattern_producesPureRotationNoTranslation() {
        // (-1, +1, -1, +1) -> pure counter-clockwise rotation, no translation.
        var v = k.forward(-1.0, 1.0, -1.0, 1.0);
        assertEquals(0.0, v.vx, EPS);
        assertEquals(0.0, v.vy, EPS);
        assertTrue(v.omega > 0, "expected positive (CCW) omega, got " + v.omega);
    }

    @Test
    void inverseThenForward_roundTripsToTheSameChassisVelocity() {
        double vx = 0.8, vy = -0.4, omega = 0.5;
        double[] powers = k.inverse(vx, vy, omega);
        var v = k.forward(powers[0], powers[1], powers[2], powers[3]);
        assertEquals(vx, v.vx, 1e-6);
        assertEquals(vy, v.vy, 1e-6);
        assertEquals(omega, v.omega, 1e-6);
    }

    @Test
    void inverse_neverCommandsPowerOutsideValidRange() {
        double[] powers = k.inverse(5.0, 5.0, 5.0); // deliberately unreachable chassis velocity
        for (double p : powers) {
            assertTrue(p >= -1.0 - 1e-9 && p <= 1.0 + 1e-9, "power out of range: " + p);
        }
    }

    @Test
    void poseIntegration_driveForwardThenTurn_matchesExpectedPath() {
        // Mirrors the MVP checkpoint scenario: "drive forward 24 inches, turn 90 degrees."
        ChassisPose pose = new ChassisPose();
        var forward = k.forward(1.0, 1.0, 1.0, 1.0); // vx = 2.0 m/s, no strafe/rotation

        double metersToTravel = 24 * 0.0254; // 24 inches
        double dt = 0.02;
        int steps = (int) Math.round(metersToTravel / (forward.vx * dt));
        for (int i = 0; i < steps; i++) {
            pose.integrate(forward, dt);
        }
        assertEquals(metersToTravel, pose.xMeters, 0.01, "x position after driving forward");
        assertEquals(0.0, pose.yMeters, 1e-9, "y should not change when driving straight");
        assertEquals(0.0, pose.headingRad, 1e-9, "heading should not change when driving straight");

        var rotateInPlace = k.forward(-1.0, 1.0, -1.0, 1.0);
        double targetHeading = Math.toRadians(90);
        int rotateSteps = (int) Math.round(targetHeading / (rotateInPlace.omega * dt));
        for (int i = 0; i < rotateSteps; i++) {
            pose.integrate(rotateInPlace, dt);
        }
        // Tolerance is one discrete integration step's worth of rotation (dt=0.02s quantizes the
        // exact step count), not kinematics error -- rounding the step count to an integer means
        // the target heading generally isn't landed on exactly.
        assertEquals(90.0, Math.toDegrees(pose.headingRad), 5.0, "heading after turning 90 degrees");
        assertEquals(metersToTravel, pose.xMeters, 0.01, "x position should not change while rotating in place");
    }
}
