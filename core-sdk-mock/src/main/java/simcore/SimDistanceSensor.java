package simcore;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Cached native scene measurement; retains a disclosed default until configured. */
public class SimDistanceSensor implements DistanceSensor {
    private final String name;
    public SimDistanceSensor(String name) { this.name = name; }
    private volatile double distanceM=1;
    public void setDistanceMeters(double value){if(Double.isNaN(value)||value<0)throw new IllegalArgumentException("Distance must be nonnegative or infinity");distanceM=value;}

    @Override public double getDistance(DistanceUnit unit) { return unit.fromMeters(distanceM); }
    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated distance sensor \"" + name + "\""; }
    @Override public void close() { }
}
