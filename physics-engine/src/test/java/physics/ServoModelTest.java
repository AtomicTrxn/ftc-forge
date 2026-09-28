package physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ServoModelTest {
    private final ServoModel.Spec spec = new ServoModel.Spec(.4, 4, Math.PI,
        8, .5, .01);

    @Test void finiteTorqueSpeedAndDeadband() {
        assertEquals(.4, ServoModel.effort(spec, 1, 0), 1e-9);
        assertEquals(-.4, ServoModel.effort(spec, -1, 0), 1e-9);
        assertEquals(0, ServoModel.effort(spec, 1, 4), 1e-9);
        assertEquals(0, ServoModel.effort(spec, .005, 0), 1e-9);
        assertThrows(IllegalArgumentException.class,
            () -> new ServoModel.Spec(-1, 4, Math.PI, 8, .5, .01));
    }
}
