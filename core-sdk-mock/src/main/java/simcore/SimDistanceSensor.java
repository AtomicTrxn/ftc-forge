package simcore;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Trivial stub -- realistic behavior needs Phase 4's field/game-piece geometry (per R1). */
public class SimDistanceSensor implements DistanceSensor {
    private final String name;
    public SimDistanceSensor(String name) { this.name = name; }

    @Override public double getDistance(DistanceUnit unit) { return unit.fromMm(1000.0); }
    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated distance sensor \"" + name + "\""; }
    @Override public void close() { }
}
