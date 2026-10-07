package physics.calibration;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** One row of R5's telemetry log schema. Per-motor fields are keyed by device name (matching R3's HardwareMap naming). */
public class TelemetryLogRow {
    public long tMs;
    public int loopIter;
    public double loopTimeMs;
    public double batteryVoltageV;
    public Double vxMps;
    public Double vyMps;
    public Double omegaRadS;
    public final Map<String, MotorSample> motors = new LinkedHashMap<>();
    /** Optional IMU/odometry readings with an independent reference; null when the row has none. */
    public SensorSample sensors;

    /**
     * Sensor readings paired with an independent reference (turntable, motion capture, a tape-measured
     * run). All references are in the same frame as the readings: start-pose frame, x forward, y left,
     * counterclockwise radians. Any value may be null (blank cell) when not recorded on that row.
     */
    public record SensorSample(Double imuYawRad, Double refYawRad, Double imuRateRadS, Double refRateRadS,
                               Double odoXM, Double odoYM, Double odoHeadingRad,
                               Double refXM, Double refYM, Double refHeadingRad) {
        public static final List<String> COLUMNS = List.of("imu_yaw_rad", "ref_yaw_rad", "imu_rate_rad_s", "ref_rate_rad_s",
            "odo_x_m", "odo_y_m", "odo_heading_rad", "ref_x_m", "ref_y_m", "ref_heading_rad");

        public Double[] values() {
            return new Double[]{imuYawRad, refYawRad, imuRateRadS, refRateRadS, odoXM, odoYM, odoHeadingRad, refXM, refYM, refHeadingRad};
        }
    }

    public static class MotorSample {
        public double power;
        public long ticks;
        public double velTps;   // ticks/sec, per R5's addition (needed to fit friction without differentiating noisy position data)
        public double currentA; // per R5: "measured current is what makes R_battery identifiable"

        public MotorSample(double power, long ticks, double velTps, double currentA) {
            this.power = power;
            this.ticks = ticks;
            this.velTps = velTps;
            this.currentA = currentA;
        }
    }
}
