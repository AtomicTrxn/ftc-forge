package simcore;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.EnumMap;
import java.util.Map;

/**
 * Backing implementation for DcMotorEx.
 *
 * DEVIATION (documented, not silent): this is a placeholder linear tick-integration, NOT
 * R4's corrected torque/current model (tau = tauStall*(V/Vnom - w/wnl)). Wiring in the real
 * motor model is explicitly Phase 3's job (per R4/Phase 3 spec) -- Phase 1 only needs to
 * prove the executor pipeline (compile -> load -> drive hardware -> observe state) works
 * end-to-end, which this placeholder is sufficient for.
 */
public class SimDcMotorEx implements DcMotorEx {

    /** Per R3/R4: the real robot-config XML can't carry gear ratio, so presets carry it here. */
    public static class MotorSpec {
        public final String sku;
        public final double ratio;
        public final double maxTicksPerSecond;
        public MotorSpec(String sku, double ratio, double maxTicksPerSecond) {
            this.sku = sku;
            this.ratio = ratio;
            this.maxTicksPerSecond = maxTicksPerSecond;
        }
    }

    private final String name;
    private final MotorSpec spec;
    private double power = 0;
    private Direction direction = Direction.FORWARD;
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;
    private double currentPositionTicks = 0;
    private int targetPosition = 0;
    private double targetVelocity = 0;
    private long lastUpdateNanos = System.nanoTime();
    private final Map<RunMode, PIDFCoefficients> pidf = new EnumMap<>(RunMode.class);

    public SimDcMotorEx(String name, MotorSpec spec) {
        this.name = name;
        this.spec = spec;
        pidf.put(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 0, 0, 0));
        pidf.put(RunMode.RUN_TO_POSITION, new PIDFCoefficients(5, 0, 0, 0));
    }

    /** Called by the executor's fixed-timestep loop -- placeholder integration, see class javadoc. */
    public void tick() {
        long now = System.nanoTime();
        double dtSeconds = (now - lastUpdateNanos) / 1_000_000_000.0;
        lastUpdateNanos = now;
        double signedPower = direction == Direction.FORWARD ? power : -power;
        double ticksPerSecond = signedPower * spec.maxTicksPerSecond;
        currentPositionTicks += ticksPerSecond * dtSeconds;
    }

    @Override public void setDirection(Direction d) { this.direction = d; }
    @Override public Direction getDirection() { return direction; }
    @Override public void setPower(double p) { this.power = Math.max(-1.0, Math.min(1.0, p)); }
    @Override public double getPower() { return power; }
    @Override public void setMode(RunMode m) { this.mode = m; if (m == RunMode.STOP_AND_RESET_ENCODER) currentPositionTicks = 0; }
    @Override public RunMode getMode() { return mode; }
    @Override public void setZeroPowerBehavior(ZeroPowerBehavior b) { this.zeroPowerBehavior = b; }
    @Override public ZeroPowerBehavior getZeroPowerBehavior() { return zeroPowerBehavior; }
    @Override public int getCurrentPosition() { return (int) Math.round(currentPositionTicks); }
    @Override public void setTargetPosition(int position) { this.targetPosition = position; }
    @Override public int getTargetPosition() { return targetPosition; }
    @Override public boolean isBusy() {
        return mode == RunMode.RUN_TO_POSITION && Math.abs(getCurrentPosition() - targetPosition) > 5;
    }

    @Override public void setVelocity(double angularRate) { this.targetVelocity = angularRate; }
    @Override public double getVelocity() { return power * spec.maxTicksPerSecond; }
    @Override public void setPIDFCoefficients(RunMode m, PIDFCoefficients p) { pidf.put(m, p); }
    @Override public PIDFCoefficients getPIDFCoefficients(RunMode m) { return pidf.get(m); }
    @Override public double getCurrent(CurrentUnit unit) { return unit.fromAmps(Math.abs(power) * 5.0); }
    @Override public void setCurrentAlert(double current, CurrentUnit unit) { }

    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated motor \"" + name + "\" (sku=" + spec.sku + ", ratio=" + spec.ratio + ")"; }
    @Override public void close() { }

    public MotorSpec getSpec() { return spec; }
}
