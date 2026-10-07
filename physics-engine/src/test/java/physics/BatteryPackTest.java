package physics;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class BatteryPackTest {
    @Test void disabledByDefaultSoVoltageIsConstantAndNothingDrains() {
        BatteryPack pack = new BatteryPack();
        pack.consume(50, 1000);
        assertEquals(12.6, pack.openCircuitV(12.6));
        assertEquals(1.0, pack.chargeFraction());
    }

    @Test void drawingChargeLowersOpenCircuitVoltageAlongTheConfiguredShape() {
        BatteryPack pack = new BatteryPack();
        pack.capacityAh = 3.0;
        pack.emptyVoltageV = 10.0;
        pack.consume(3.0, 1800);                      // 1.5 Ah of 3 Ah
        assertEquals(0.5, pack.chargeFraction(), 1e-9);
        assertEquals(11.3, pack.openCircuitV(12.6), 1e-9);
        pack.shape = 2;
        assertEquals(10.65, pack.openCircuitV(12.6), 1e-9); // 10 + 2.6 * 0.25
        pack.consume(100, 1e6);
        assertEquals(0, pack.chargeFraction());
        assertEquals(10.0, pack.openCircuitV(12.6), 1e-9);
    }

    @Test void brownoutLatchesHoldsAndRecoversWithHysteresis() {
        BatteryPack pack = new BatteryPack();
        pack.brownoutV = 7;
        pack.brownoutHoldS = 2;
        pack.brownoutRecoveryV = 9;
        pack.updateBrownout(8, 12, 0);
        assertFalse(pack.isBrownedOut());
        pack.updateBrownout(6.9, 12, 1);
        assertTrue(pack.isBrownedOut());
        pack.updateBrownout(12, 12, 2);               // load removed but hold not elapsed
        assertTrue(pack.isBrownedOut());
        pack.updateBrownout(8, 8.5, 3.5);             // hold elapsed, open-circuit below recovery voltage
        assertTrue(pack.isBrownedOut());
        pack.updateBrownout(12, 12, 4);
        assertFalse(pack.isBrownedOut());
    }

    @Test void extraLoadSagsTheBatteryByRTimesCurrentAndMatchesTheNoMotorCase() {
        BatteryModel model = new BatteryModel(12.6, 0.15);
        assertEquals(12.6, model.solveBatteryVoltage(List.of()), 1e-12);
        assertEquals(12.6 - 0.15 * 2.0, model.solveBatteryVoltage(List.of(), 2.0, 12.6), 1e-12);
        assertEquals(11.0, model.solveBatteryVoltage(List.of(), 0.0, 11.0), 1e-12);
    }
}
