package simcore;

import physics.MotorSpec;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Hand-authored preset robot config (Phase 1 scope, NOT R6's full CAD/URDF importer --
 * that's Phase 5). Carries a per-motor SKU/ratio field the real robot-config XML can't
 * express (per R3/R4's finding), plus the real datasheet constants R4's motor model
 * (Phase 3) needs: stall torque/current, no-load speed, nominal voltage, encoder CPR.
 */
public class PresetRobotConfig {
    public String name;
    public final Map<String, MotorSpec> motors = new LinkedHashMap<>();

    @SuppressWarnings("unchecked")
    public static PresetRobotConfig load(Path path) throws IOException {
        String json = Files.readString(path);
        Map<String, Object> root = MiniJson.parseObject(json);
        PresetRobotConfig config = new PresetRobotConfig();
        config.name = (String) root.get("name");
        Map<String, Object> motors = (Map<String, Object>) root.get("motors");
        if (motors != null) {
            for (Map.Entry<String, Object> e : motors.entrySet()) {
                Map<String, Object> m = (Map<String, Object>) e.getValue();
                String sku = (String) m.get("sku");
                double ratio = ((Number) m.get("ratio")).doubleValue();
                double tauStallNm = ((Number) m.get("tauStallNm")).doubleValue();
                double iStallAmps = ((Number) m.get("iStallAmps")).doubleValue();
                double omegaNoLoadRadS = ((Number) m.get("omegaNoLoadRadS")).doubleValue();
                double vNominal = ((Number) m.get("vNominal")).doubleValue();
                double encoderCountsPerRev = ((Number) m.get("encoderCountsPerRev")).doubleValue();
                config.motors.put(e.getKey(), new MotorSpec(
                    sku, ratio, tauStallNm, iStallAmps, omegaNoLoadRadS, vNominal, encoderCountsPerRev));
            }
        }
        return config;
    }
}
