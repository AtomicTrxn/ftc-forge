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
    public final List<String> extraClasspath = new ArrayList<>();

    @SuppressWarnings("unchecked")
    public static SimConfig load(Path projectRoot) throws IOException {
        Path configPath = projectRoot.resolve("sim.config");
        Map<String, Object> root = MiniJson.parseObject(Files.readString(configPath));
        SimConfig config = new SimConfig();
        if (root.containsKey("sourceRoot")) config.sourceRoot = (String) root.get("sourceRoot");
        if (root.containsKey("robotConfig")) config.robotConfig = (String) root.get("robotConfig");
        if (root.containsKey("presetMotors")) config.presetMotors = (String) root.get("presetMotors");
        if (root.containsKey("extraClasspath")) {
            for (Object o : (List<Object>) root.get("extraClasspath")) {
                config.extraClasspath.add((String) o);
            }
        }
        return config;
    }
}
