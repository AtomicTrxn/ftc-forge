package simcore;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Backing implementation for IMU. Static/zero orientation in Phase 1 -- real orientation
 * tracking from chassis kinematics is Phase 2's job (R2's kinematics model), not this phase's.
 */
public class SimIMU implements IMU {
    private final String name;
    private Parameters parameters;

    public SimIMU(String name) { this.name = name; }

    @Override public boolean initialize(Parameters parameters) {
        this.parameters = parameters;
        return true;
    }

    @Override public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return new YawPitchRollAngles(0, 0, 0, System.nanoTime());
    }

    @Override public AngularVelocity getRobotAngularVelocity(AngleUnit unit) {
        return new AngularVelocity(unit, 0, 0, 0);
    }

    @Override public void resetYaw() { }

    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated IMU \"" + name + "\""; }
    @Override public void close() { }
}
