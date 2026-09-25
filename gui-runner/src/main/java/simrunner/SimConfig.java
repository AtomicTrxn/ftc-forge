package simrunner;

import simcore.MiniJson;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Parses a team project's sim.config marker file (per R3's design): the minimal, low-friction
 * "point the simulator at your folder" mechanism -- no entry-OpMode field needed, since
 * discovery is annotation-driven (see OpModeDiscovery).
 */
public class SimConfig {
    public String sourceRoot = "TeamCode/src/main/java";
    public String robotConfig;
    public String presetMotors;
    public String urdf;
    public Double totalMassKg;
    public int vhacdMaxHulls = 8;
    public final List<String> extraClasspath = new ArrayList<>();

    @SuppressWarnings("unchecked")
    public static SimConfig load(Path projectRoot) throws IOException {
        Path configPath = projectRoot.resolve("sim.config");
        Map<String, Object> root = MiniJson.parseObject(Files.readString(configPath));
        SimConfig config = new SimConfig();
        if (root.containsKey("sourceRoot")) config.sourceRoot = (String) root.get("sourceRoot");
        if (root.containsKey("robotConfig")) config.robotConfig = (String) root.get("robotConfig");
        if (root.containsKey("presetMotors")) config.presetMotors = (String) root.get("presetMotors");
        if (root.containsKey("urdf")) config.urdf = (String) root.get("urdf");
        if (root.containsKey("total_mass_kg")) config.totalMassKg = ((Number) root.get("total_mass_kg")).doubleValue();
        if (root.containsKey("vhacd_max_hulls")) config.vhacdMaxHulls = ((Number) root.get("vhacd_max_hulls")).intValue();
        if (config.vhacdMaxHulls < 1 || config.vhacdMaxHulls > 16)
            throw new IllegalArgumentException("vhacd_max_hulls must be between 1 and 16");
        if (root.containsKey("extraClasspath")) {
            for (Object o : (List<Object>) root.get("extraClasspath")) {
                config.extraClasspath.add((String) o);
            }
        }
        return config;
    }
}
