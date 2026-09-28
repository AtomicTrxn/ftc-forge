package simrunner;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class PrincipalInertiaTest {
    @Test void diagonalizationRetainsCrossTermsAndBulletFrame() {
        PrincipalInertia inertia = new PrincipalInertia();
        inertia.addRotated(new double[][] {{2, .6, .2}, {.6, 3, -.1}, {.2, -.1, 4}}, new Quaternion());
        inertia.addParallelAxis(1.5, new Vector3f(.3f, -.2f, .1f));
        var principal = inertia.diagonalize("test");
        var rotation = principal.rotation().toRotationMatrix();
        float[] moments = {principal.moments().x, principal.moments().y, principal.moments().z};
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
            double reconstructed = 0;
            for (int k = 0; k < 3; k++) reconstructed += rotation.get(i, k) * moments[k] * rotation.get(j, k);
            assertEquals(inertia.get(i, j), reconstructed, 1e-5);
        }
        assertTrue(Math.abs(inertia.get(0, 1)) > .5);
    }

    @Test void rejectsIndefiniteTensorEvenWithPositiveDiagonal() {
        PrincipalInertia inertia = new PrincipalInertia();
        inertia.addRotated(new double[][] {{1, 2, 0}, {2, 1, 0}, {0, 0, 1}}, new Quaternion());
        assertThrows(IllegalArgumentException.class, () -> inertia.diagonalize("bad"));
    }
}
