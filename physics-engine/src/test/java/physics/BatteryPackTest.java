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

    @Test void resistanceRisesLinearlyWithDepletionOnlyWhenConfigured() {
        BatteryPack pack = new BatteryPack();
        pack.capacityAh = 3;
        assertEquals(0.15, pack.resistanceOhm(0.15));           // no empty value: unchanged
        pack.emptyResistanceOhm = 0.55;
        assertEquals(0.15, pack.resistanceOhm(0.15), 1e-12);    // full
        pack.setChargeFraction(0.5);
        assertEquals(0.35, pack.resistanceOhm(0.15), 1e-12);
        pack.setChargeFraction(0);
        assertEquals(0.55, pack.resistanceOhm(0.15), 1e-12);
        pack.capacityAh = 0;
        assertEquals(0.15, pack.resistanceOhm(0.15), 1e-12);    // tracking off
    }

    @Test void coldPackHasHigherResistanceAndDefaultsChangeNothing() {
        BatteryPack pack = new BatteryPack();
        pack.ambientC = 0;
        assertEquals(0.15, pack.resistanceOhm(0.15), 1e-12, "no coefficient: temperature has no effect");
        pack.resistanceTempCoeffPerK = 0.01;
        assertEquals(0.15 * 1.25, pack.resistanceOhm(0.15), 1e-12);
        pack.ambientC = 25;
        assertEquals(0.15, pack.resistanceOhm(0.15), 1e-12);
        pack.ambientC = 100;
        assertEquals(0.15 * 0.3, pack.resistanceOhm(0.15), 1e-12, "floored at 30% of the base value");
    }

    @Test void selfHeatingFollowsTheFirstOrderCurveAndCoolsBack() {
        BatteryPack pack = new BatteryPack();
        pack.thermalCapacityJPerK = 100;
        pack.thermalResistanceKPerW = 2;
        double steady = 25 + 10 * 10 * 0.2 * 2;
        pack.heat(10, 0.2, 50);
        assertEquals(steady + (25 - steady) * Math.exp(-50 / 200.0), pack.temperatureC(), 1e-9);
        for (int i = 0; i < 1000; i++) pack.heat(0, 0.2, 10);
        assertEquals(25, pack.temperatureC(), 1e-6);
        pack.thermalCapacityJPerK = 0;
        pack.resetTemperature();
        pack.heat(50, 1, 100);
        assertEquals(25, pack.temperatureC(), "no heat capacity configured: temperature stays at ambient");
    }
}
