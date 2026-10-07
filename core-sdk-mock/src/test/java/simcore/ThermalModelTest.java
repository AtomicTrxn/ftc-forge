package simcore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Winding and pack temperature, and distance/color sensor bus cost, through the sim.config path. */
class ThermalModelTest {
    @TempDir Path dir;

    private HardwareMap robot(String config) throws Exception {
        Path xml = dir.resolve("robot.xml");
        Files.writeString(xml, "<Robot><LynxUsbDevice name=\"u\"><LynxModule name=\"Hub\">"
            + "<goBILDA5202SeriesMotor name=\"m\" port=\"0\"/><Distance name=\"dist\" port=\"0\"/><ColorSensor name=\"color\" port=\"1\"/>"
            + "</LynxModule></LynxUsbDevice></Robot>");
        Path preset = dir.resolve("preset.json");
        Files.writeString(preset, "{\"name\":\"p\",\"motors\":{\"m\":{\"sku\":\"t\",\"ratio\":19.2,\"tauStallNm\":2.4,\"iStallAmps\":9.2,\"omegaNoLoadRadS\":32.7,\"vNominal\":12,\"encoderCountsPerRev\":537.7}}}");
        HardwareMap map = HardwareMapBuilder.build(RobotConfigXml.parse(xml.toFile()), PresetRobotConfig.load(preset));
        ElectricalConfig.parse(MiniJson.parseObject(config)).apply(map);
        return map;
    }

    @Test void stalledMotorHeatsLosesTorqueAndCoolsAgain() throws Exception {
        HardwareMap map = robot("{\"motor_defaults\":{\"thermal_capacity_j_per_k\":20,\"thermal_resistance_k_per_w\":4,"
            + "\"derate_start_c\":40,\"derate_full_c\":90,\"thermal_max_derate\":0.4}}");
        SimDcMotorEx motor = map.get(SimDcMotorEx.class, "m");
        motor.useExternalShaft();
        motor.setPower(1);
        long t = 0;
        motor.syncExternalShaft(0, 0);
        double coldTorque = motor.externalShaftTorque();
        for (int i = 0; i < 3000; i++) {                      // 30 s held at stall
            motor.syncExternalShaft(0, 0);
            motor.integrate(12, 0.01, t += 10);
        }
        assertTrue(motor.getWindingTempC() > 90, "temperature " + motor.getWindingTempC());
        motor.syncExternalShaft(0, 0);
        assertTrue(motor.externalShaftTorque() < coldTorque * 0.7, "hot torque " + motor.externalShaftTorque() + " vs cold " + coldTorque);
        motor.setPower(0);
        for (int i = 0; i < 60000; i++) {                     // 10 minutes unpowered
            motor.syncExternalShaft(0, 0);
            motor.integrate(12, 0.01, t += 10);
        }
        assertEquals(25, motor.getWindingTempC(), 0.5);
    }

    @Test void windingModelIsOffByDefaultAndPerMotorOverridesApply() throws Exception {
        HardwareMap off = robot("{}");
        SimDcMotorEx motor = off.get(SimDcMotorEx.class, "m");
        motor.useExternalShaft();
        motor.setPower(1);
        for (int i = 1; i <= 1000; i++) { motor.syncExternalShaft(0, 0); motor.integrate(12, 0.01, i * 10L); }
        assertEquals(25, motor.getWindingTempC(), 0.0);

        HardwareMap on = robot("{\"motor_overrides\":{\"m\":{\"thermal_capacity_j_per_k\":20,\"ambient_c\":10}}}");
        assertEquals(10, on.get(SimDcMotorEx.class, "m").getWindingTempC(), 0.0);
    }

    @Test void invalidThermalSettingsAreRejected() {
        for (String bad : List.of("{\"motor_defaults\":{\"thermal_resistance_k_per_w\":0}}",
                "{\"motor_defaults\":{\"derate_start_c\":90,\"derate_full_c\":80}}",
                "{\"motor_defaults\":{\"resistance_tempco_per_k\":1}}", "{\"motor_defaults\":{\"ambient_c\":500}}",
                "{\"battery\":{\"thermal_resistance_k_per_w\":0}}", "{\"battery\":{\"resistance_temp_coeff_per_k\":2}}",
                "{\"battery\":{\"ambient_c\":100}}", "{\"bus\":{\"sensor_read_cost_ms\":99}}"))
            assertThrows(IllegalArgumentException.class, () -> ElectricalConfig.parse(MiniJson.parseObject(bad)), bad);
    }

    @Test void coldPackSagsHarderThanAWarmOne() throws Exception {
        long t = 0;
        double[] volts = new double[2];
        String[] ambients = {"25", "0"};
        for (int i = 0; i < 2; i++) {
            HardwareMap map = robot("{\"battery\":{\"ambient_c\":" + ambients[i] + ",\"resistance_temp_coeff_per_k\":0.01},"
                + "\"loads\":{\"hub_baseline_a\":5}}");
            HardwareMapBuilder.tickMotors(map, 0.01, t += 10);
            volts[i] = map.get(com.qualcomm.hardware.lynx.LynxModule.class, "Hub").getVoltage();
        }
        assertEquals(12.6 - 0.15 * 5, volts[0], 1e-9);
        assertEquals(12.6 - 0.15 * 1.25 * 5, volts[1], 1e-9);
    }

    @Test void heavyLoadWarmsThePackAndRaisesNothingWhenHeatCapacityIsUnset() throws Exception {
        HardwareMap map = robot("{\"battery\":{\"thermal_capacity_j_per_k\":50,\"thermal_resistance_k_per_w\":3,"
            + "\"resistance_temp_coeff_per_k\":0.01},\"loads\":{\"hub_baseline_a\":20}}");
        long t = 0;
        double first = 0, last = 0;
        for (int i = 0; i < 6000; i++) {
            HardwareMapBuilder.tickMotors(map, 0.01, t += 10);
            double v = map.get(com.qualcomm.hardware.lynx.LynxModule.class, "Hub").getVoltage();
            if (i == 0) first = v;
            last = v;
        }
        assertTrue(HardwareMapBuilder.getBatteryPack().temperatureC() > 40, "pack " + HardwareMapBuilder.getBatteryPack().temperatureC());
        assertTrue(last > first, "a warming pack has lower resistance, so the sag eases: " + first + " -> " + last);

        HardwareMap off = robot("{\"loads\":{\"hub_baseline_a\":20}}");
        for (int i = 0; i < 100; i++) HardwareMapBuilder.tickMotors(off, 0.01, 10L * (i + 1));
        assertEquals(25, HardwareMapBuilder.getBatteryPack().temperatureC(), 0.0);
    }

    @Test void distanceAndColorReadsBlockForTheConfiguredBusCost() throws Exception {
        HardwareMap map = robot("{\"bus\":{\"sensor_read_cost_ms\":5}}");
        var distance = map.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "dist");
        var color = map.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "color");
        long start = System.nanoTime();
        for (int i = 0; i < 3; i++) { distance.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM); color.red(); }
        assertTrue((System.nanoTime() - start) / 1e6 >= 28, "6 reads at 5 ms");
        HardwareMap free = robot("{}");
        start = System.nanoTime();
        for (int i = 0; i < 1000; i++) free.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "color").red();
        assertTrue((System.nanoTime() - start) / 1e6 < 50);
    }
}
