package simcore;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import physics.MotorModel;
import physics.MotorSpec;
import physics.SensorRingBuffer;

import java.util.EnumMap;
import java.util.Map;

/**
 * Backing implementation for DcMotorEx, now wired to R4's corrected motor model (Phase 3) --
 * replacing Phase 1's placeholder linear tick integration.
 *
 * Rotational inertia is a fixed placeholder (ROTATIONAL_INERTIA_KG_M2), not sourced from
 * real CAD -- Phase 5's URDF importer is where a real per-robot inertia would come from.
 * Encoder ticks are read through a SensorRingBuffer to model I2C read latency (R4's
 * reassessment: cheap enough to ship in Phase 3 rather than deferring further).
 */
public class SimDcMotorEx implements DcMotorEx {

    public static final double ROTATIONAL_INERTIA_KG_M2 = 0.0015; // placeholder; see class javadoc
    public static long encoderLatencyMs = 8; // per R4's 7-10ms I2C default

    private final String name;
    private final MotorSpec spec;
    private final MotorModel motorModel = new MotorModel();

    private double power = 0;
    private Direction direction = Direction.FORWARD;
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;

    private double omegaRadS = 0;
    private double currentPositionTicks = 0;
    private int targetPosition = 0;
    private double targetVelocityFraction = 0;
    private double lastCurrentAmps = 0;

    private final Map<RunMode, PIDFCoefficients> pidf = new EnumMap<>(RunMode.class);
    private final SensorRingBuffer<Integer> tickBuffer = new SensorRingBuffer<>(200);
    private int lastKnownTicks = 0;

    public SimDcMotorEx(String name, MotorSpec spec) {
        this.name = name;
        this.spec = spec;
        pidf.put(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.0, 0, 0, 0));
        pidf.put(RunMode.RUN_TO_POSITION, new PIDFCoefficients(0.01, 0, 0, 0));
    }

    /** Pure function of current state -- safe to call twice per tick (see HardwareMapBuilder). */
    public double commandedPower() {
        switch (mode) {
            case RUN_USING_ENCODER: {
                double targetOmega = targetVelocityFraction * spec.omegaNoLoadRadS;
                double kP = pidf.get(RunMode.RUN_USING_ENCODER).p;
                double correction = kP * (targetOmega - omegaRadS) / spec.omegaNoLoadRadS;
                return clamp(power + correction);
            }
            case RUN_TO_POSITION: {
                double kPPos = pidf.get(RunMode.RUN_TO_POSITION).p;
                double velocityTarget = clamp(kPPos * (targetPosition - currentPositionTicks));
                double targetOmega = velocityTarget * spec.omegaNoLoadRadS;
                double kPVel = pidf.get(RunMode.RUN_USING_ENCODER).p;
                double correction = kPVel * (targetOmega - omegaRadS) / spec.omegaNoLoadRadS;
                return clamp(velocityTarget + correction);
            }
            default:
                return power;
        }
    }

    /** Called once per tick by HardwareMapBuilder, after the shared battery voltage has been solved. */
    public void integrate(double batteryVoltage, double dtSeconds, long simTimeMs) {
        double signedPower = direction == Direction.FORWARD ? commandedPower() : -commandedPower();
        double vActual = signedPower * batteryVoltage;

        double tau = motorModel.torque(spec, omegaRadS, vActual);
        double alpha = tau / ROTATIONAL_INERTIA_KG_M2;
        omegaRadS += alpha * dtSeconds;

        lastCurrentAmps = motorModel.current(spec, omegaRadS, vActual);
        motorModel.tickThermal(spec, lastCurrentAmps, dtSeconds);

        double revolutions = (omegaRadS * dtSeconds) / (2 * Math.PI);
        currentPositionTicks += revolutions * spec.encoderCountsPerRev;

        tickBuffer.push(simTimeMs, (int) Math.round(currentPositionTicks));
        Integer delayed = tickBuffer.read(simTimeMs, encoderLatencyMs);
        if (delayed != null) lastKnownTicks = delayed;
    }

    private static double clamp(double v) { return Math.max(-1.0, Math.min(1.0, v)); }

    @Override public void setDirection(Direction d) { this.direction = d; }
    @Override public Direction getDirection() { return direction; }
    @Override public void setPower(double p) {
        this.power = clamp(p);
        if (mode == RunMode.RUN_USING_ENCODER) this.targetVelocityFraction = this.power;
    }
    @Override public double getPower() { return power; }
    @Override public void setMode(RunMode m) {
        this.mode = m;
        if (m == RunMode.STOP_AND_RESET_ENCODER) { currentPositionTicks = 0; omegaRadS = 0; lastKnownTicks = 0; }
    }
    @Override public RunMode getMode() { return mode; }
    @Override public void setZeroPowerBehavior(ZeroPowerBehavior b) { this.zeroPowerBehavior = b; }
    @Override public ZeroPowerBehavior getZeroPowerBehavior() { return zeroPowerBehavior; }
    @Override public int getCurrentPosition() { return lastKnownTicks; } // per R4: reads latency-delayed value, not the true instantaneous one
    @Override public void setTargetPosition(int position) { this.targetPosition = position; }
    @Override public int getTargetPosition() { return targetPosition; }
    @Override public boolean isBusy() {
        return mode == RunMode.RUN_TO_POSITION && Math.abs(getCurrentPosition() - targetPosition) > 5;
    }

    @Override public void setVelocity(double angularRate) {
        this.targetVelocityFraction = angularRate / spec.omegaNoLoadRadS;
    }
    @Override public double getVelocity() { return (omegaRadS / (2 * Math.PI)) * spec.encoderCountsPerRev; }
    @Override public void setPIDFCoefficients(RunMode m, PIDFCoefficients p) { pidf.put(m, p); }
    @Override public PIDFCoefficients getPIDFCoefficients(RunMode m) { return pidf.get(m); }
    @Override public double getCurrent(CurrentUnit unit) { return unit.fromAmps(Math.abs(lastCurrentAmps)); }
    @Override public void setCurrentAlert(double current, CurrentUnit unit) { }

    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated motor \"" + name + "\" (sku=" + spec.sku + ", ratio=" + spec.ratio + ")"; }
    @Override public void close() { }

    public MotorSpec getSpec() { return spec; }
    public double getOmegaRadS() { return omegaRadS; }
    public double getRawTicks() { return currentPositionTicks; }
}
