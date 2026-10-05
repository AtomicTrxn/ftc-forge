package simrunner;

import java.nio.file.Path;

/** CLI process is the isolation boundary for TeamCode that ignores cooperative stop. */
public final class HeadlessPhysicsMain {
    public static void main(String[] args) throws Exception {
        if (args.length < 2) throw new IllegalArgumentException("Usage: HeadlessPhysicsMain <projectDir> <OpMode> [--duration seconds] [--watchdog-ms ms] [--settle-seconds seconds] [--max-lag-ms ms] [--report path.json]");
        Path project = Path.of(args[0]).toAbsolutePath();
        Path report = project.resolve("build/headless-physics/report.json");
        double duration = 10, settle = .25;
        Long watchdog = null;
        long lag = 250;
        for (int i = 2; i < args.length; i++) {
            String flag = args[i];
            if (i + 1 == args.length) throw new IllegalArgumentException("Missing value for " + flag);
            String value = args[++i];
            switch (flag) {
                case "--duration" -> duration = Double.parseDouble(value);
                case "--watchdog-ms" -> watchdog = Long.parseLong(value);
                case "--settle-seconds" -> settle = Double.parseDouble(value);
                case "--max-lag-ms" -> lag = Long.parseLong(value);
                case "--report" -> report = Path.of(value).toAbsolutePath();
                default -> throw new IllegalArgumentException("Unknown argument " + flag);
            }
        }
        var options = new HeadlessPhysicsRunner.Options(duration, watchdog == null ? Math.round(duration * 1000) + 2000 : watchdog, settle, lag);
        var result = HeadlessPhysicsRunner.run(project, args[1], report, options);
        System.out.println("[HEADLESS PHYSICS] " + result.get("outcome") + " | report: " + report);
        if (!Boolean.TRUE.equals(result.get("success"))) throw new IllegalStateException("Headless run failed: " + result.get("outcome") + " " + result.getOrDefault("failure", ""));
    }
}
