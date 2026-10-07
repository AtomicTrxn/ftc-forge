package simcore;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

/** Bulk-caching modes and per-read bus cost on a hub. */
class HubBusTest {
    @TempDir Path dir;

    private HardwareMap robot(String config) throws Exception {
        Path xml = dir.resolve("robot.xml");
        Files.writeString(xml, "<Robot><LynxUsbDevice name=\"u\"><LynxModule name=\"Hub1\">"
            + "<goBILDA5202SeriesMotor name=\"a\" port=\"0\"/><goBILDA5202SeriesMotor name=\"b\" port=\"1\"/></LynxModule>"
            + "<LynxModule name=\"Hub2\"><goBILDA5202SeriesMotor name=\"c\" port=\"0\"/></LynxModule></LynxUsbDevice></Robot>");
        Path preset = dir.resolve("preset.json");
        String spec = "{\"sku\":\"t\",\"ratio\":19.2,\"tauStallNm\":2.4,\"iStallAmps\":9.2,\"omegaNoLoadRadS\":32.7,\"vNominal\":12,\"encoderCountsPerRev\":537.7}";
        Files.writeString(preset, "{\"name\":\"p\",\"motors\":{\"a\":" + spec + ",\"b\":" + spec + ",\"c\":" + spec + "}}");
        HardwareMap map = HardwareMapBuilder.build(RobotConfigXml.parse(xml.toFile()), PresetRobotConfig.load(preset));
        ElectricalConfig.parse(MiniJson.parseObject(config)).apply(map);
        return map;
    }

    private static long clock;
    private static void spin(HardwareMap map, int ticks) {
        for (int i = 0; i < ticks; i++) { clock += 10; HardwareMapBuilder.tickMotors(map, 0.01, clock); }
    }

    @Test void offModeEveryReadIsABusTransactionAndValuesAreLive() throws Exception {
        clock = 0;
        HardwareMap map = robot("{\"encoder_latency_ms\":0}");
        var hub = map.get(SimVoltageSensor.class, "Hub1");
        var a = map.get(DcMotor.class, "a");
        a.setPower(1);
        spin(map, 10);
        int first = a.getCurrentPosition();
        spin(map, 10);
        assertTrue(a.getCurrentPosition() > first);
        assertEquals(2, hub.busTransactions());
    }

    @Test void autoModeSharesOneSnapshotUntilTheSameValueIsReadAgain() throws Exception {
        clock = 0;
        HardwareMap map = robot("{\"encoder_latency_ms\":0}");
        var hub = map.get(SimVoltageSensor.class, "Hub1");
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        map.get(DcMotor.class, "a").setPower(1);
        map.get(DcMotor.class, "b").setPower(1);
        spin(map, 10);
        var a = map.get(DcMotor.class, "a");
        var b = map.get(DcMotor.class, "b");
        int fresh = a.getCurrentPosition();                // refresh: one transaction
        b.getCurrentPosition();                            // served from that snapshot
        ((com.qualcomm.robotcore.hardware.DcMotorEx) a).getVelocity();                                   // different value of the same motor: still the snapshot
        assertEquals(1, hub.busTransactions());
        spin(map, 10);
        assertTrue(a.getCurrentPosition() > fresh, "a second read of the same value refreshes the cache");
        assertEquals(2, hub.busTransactions());
    }

    @Test void manualModeReturnsStaleDataUntilTheCacheIsCleared() throws Exception {
        clock = 0;
        HardwareMap map = robot("{\"encoder_latency_ms\":0}");
        var hub = map.get(SimVoltageSensor.class, "Hub1");
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        var a = map.get(DcMotor.class, "a");
        a.setPower(1);
        spin(map, 10);
        int cached = a.getCurrentPosition();
        spin(map, 20);
        assertEquals(cached, a.getCurrentPosition(), "forgot clearBulkCache(): stale");
        hub.clearBulkCache();
        assertTrue(a.getCurrentPosition() > cached);
        assertEquals(2, hub.busTransactions());
    }

    @Test void eachHubKeepsItsOwnCache() throws Exception {
        clock = 0;
        HardwareMap map = robot("{\"encoder_latency_ms\":0}");
        map.get(SimVoltageSensor.class, "Hub1").setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        map.get(DcMotor.class, "a").setPower(1);
        map.get(DcMotor.class, "c").setPower(1);
        spin(map, 10);
        int a0 = map.get(DcMotor.class, "a").getCurrentPosition();
        int c0 = map.get(DcMotor.class, "c").getCurrentPosition();
        spin(map, 20);
        assertEquals(a0, map.get(DcMotor.class, "a").getCurrentPosition());          // Hub1 cached
        assertTrue(map.get(DcMotor.class, "c").getCurrentPosition() > c0);             // Hub2 uncached
    }

    @Test void readCostBlocksTheCallerPerTransaction() throws Exception {
        clock = 0;
        HardwareMap map = robot("{\"bus\":{\"read_cost_ms\":5}}");
        var a = map.get(DcMotor.class, "a");
        long start = System.nanoTime();
        for (int i = 0; i < 6; i++) a.getCurrentPosition();
        double elapsedMs = (System.nanoTime() - start) / 1e6;
        assertTrue(elapsedMs >= 28, "6 reads at 5 ms should take about 30 ms, took " + elapsedMs);

        var hub = map.get(SimVoltageSensor.class, "Hub1");
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        start = System.nanoTime();
        for (int i = 0; i < 50; i++) a.getCurrentPosition();
        assertTrue((System.nanoTime() - start) / 1e6 < 25, "cached reads are free after the first");
    }

    @Test void defaultsKeepReadsFreeAndLive() throws Exception {
        clock = 0;
        HardwareMap map = robot("{}");
        var a = map.get(DcMotor.class, "a");
        long start = System.nanoTime();
        for (int i = 0; i < 1000; i++) a.getCurrentPosition();
        assertTrue((System.nanoTime() - start) / 1e6 < 50);
    }
}
