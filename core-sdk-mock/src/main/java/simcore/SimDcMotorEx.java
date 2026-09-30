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
 * Unbound motors use a placeholder rotor/load inertia. Imported articulated mechanisms
 * instead receive actual shaft motion from Bullet and supply electrical torque to the joint.
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
    private boolean externallyDriven;
    private double externalShaftRadians;
    private double encoderZeroTicks;
    private double batteryVoltage = 12;

    private final Map<RunMode, PIDFCoefficients> pidf = new EnumMap<>(RunMode.class);
    private SensorRingBuffer<Integer> tickBuffer = new SensorRingBuffer<>(200);
    private int lastKnownTicks = 0;

    public SimDcMotorEx(String name, MotorSpec spec) {
        this.name = name;
        this.spec = spec;
        pidf.put(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.0, 0, 0, 0));
        pidf.put(RunMode.RUN_TO_POSITION, new PIDFCoefficients(0.01, 0, 0, 0));
    }

    /** Pure function of current state -- safe to call twice per tick (see HardwareMapBuilder). */
    public synchronized double commandedPower() {
        double logicalOmega = direction == Direction.REVERSE ? -omegaRadS : omegaRadS;
        switch (mode) {
            case STOP_AND_RESET_ENCODER: return 0;
            case RUN_USING_ENCODER: {
                double targetOmega = targetVelocityFraction * spec.omegaNoLoadRadS;
                double kP = pidf.get(RunMode.RUN_USING_ENCODER).p;
                double correction = kP * (targetOmega - logicalOmega) / spec.omegaNoLoadRadS;
                return clamp(power + correction);
            }
            case RUN_TO_POSITION: {
                double kPPos = pidf.get(RunMode.RUN_TO_POSITION).p;
                double velocityTarget = clamp(kPPos * (targetPosition - currentPositionTicks));
                double targetOmega = velocityTarget * spec.omegaNoLoadRadS;
                double kPVel = pidf.get(RunMode.RUN_USING_ENCODER).p;
                double correction = kPVel * (targetOmega - logicalOmega) / spec.omegaNoLoadRadS;
                return clamp(velocityTarget + correction);
            }
            default:
                return power;
        }
    }

    public synchronized void configureFriction(double staticTorqueNm, double viscousBNmS) {
        if (!Double.isFinite(staticTorqueNm) || staticTorqueNm < 0
            || !Double.isFinite(viscousBNmS) || viscousBNmS < 0)
            throw new IllegalArgumentException("Motor friction must be finite and nonnegative");
        motorModel.tauStaticNm = staticTorqueNm;
        motorModel.viscousBNms = viscousBNmS;
    }

    public synchronized double signedCommandedPower() {
        return commandedPower() * (direction == Direction.FORWARD ? 1 : -1);
    }

    /** Called once per tick by HardwareMapBuilder, after the shared battery voltage has been solved. */
    public synchronized void integrate(double batteryVoltage, double dtSeconds, long simTimeMs) {
        this.batteryVoltage = batteryVoltage;
        double signedPower = direction == Direction.FORWARD ? commandedPower() : -commandedPower();
        double vActual = signedPower * batteryVoltage;

        double tau = motorModel.torque(spec, omegaRadS, vActual);
        double alpha = tau / ROTATIONAL_INERTIA_KG_M2;
        if (!externallyDriven) omegaRadS += alpha * dtSeconds;

        lastCurrentAmps = externallyDriven && (mode == RunMode.STOP_AND_RESET_ENCODER ||
            (signedPower == 0 && zeroPowerBehavior == ZeroPowerBehavior.FLOAT)) ? 0 : motorModel.current(spec, omegaRadS, vActual);
        motorModel.tickThermal(spec, lastCurrentAmps, dtSeconds);

        if (!externallyDriven) {
            externalShaftRadians += omegaRadS * dtSeconds;
            currentPositionTicks = shaftTicks() - encoderZeroTicks;
        }

        tickBuffer.push(simTimeMs, (int) Math.round(currentPositionTicks));
        Integer delayed = tickBuffer.read(simTimeMs, encoderLatencyMs);
        if (delayed != null) lastKnownTicks = delayed;
    }

    /** Bind a real physics joint: the motor model supplies effort, Bullet supplies motion. */
    public synchronized void useExternalShaft() { externallyDriven = true; }

    public synchronized void syncExternalShaft(double radians, double radiansPerSecond) {
        if (!externallyDriven) throw new IllegalStateException("Motor is not bound to external physics");
        externalShaftRadians = radians;
        omegaRadS = radiansPerSecond;
        currentPositionTicks = shaftTicks() - encoderZeroTicks;
    }

    private double shaftTicks() {
        return externalShaftRadians * spec.encoderCountsPerRev / (2 * Math.PI)
            * (direction == Direction.FORWARD ? 1 : -1);
    }

    /** Physical output-shaft torque, including back-EMF and the BRAKE/FLOAT distinction. */
    public synchronized double externalShaftTorque() {
        if (mode == RunMode.STOP_AND_RESET_ENCODER) return 0;
        double command = commandedPower();
        if (command == 0 && zeroPowerBehavior == ZeroPowerBehavior.FLOAT) return 0;
        double voltage = command * batteryVoltage * (direction == Direction.FORWARD ? 1 : -1);
        return motorModel.torque(spec, omegaRadS, voltage);
    }

    public synchronized double externalTorqueDamping() {
        if (mode == RunMode.STOP_AND_RESET_ENCODER ||
            (commandedPower() == 0 && zeroPowerBehavior == ZeroPowerBehavior.FLOAT)) return 0;
        return spec.tauStallNm / spec.omegaNoLoadRadS;
    }

    private static double clamp(double v) { return Math.max(-1.0, Math.min(1.0, v)); }

    @Override public synchronized void setDirection(Direction d) {
        if (direction != d) {
            encoderZeroTicks = -encoderZeroTicks;
            currentPositionTicks = -currentPositionTicks;
            lastKnownTicks = -lastKnownTicks;
            tickBuffer = new SensorRingBuffer<>(200);
        }
        this.direction = d;
    }
    @Override public synchronized Direction getDirection() { return direction; }
    @Override public synchronized void setPower(double p) {
        this.power = clamp(p);
        if (mode == RunMode.RUN_USING_ENCODER) this.targetVelocityFraction = this.power;
    }
    @Override public synchronized double getPower() { return power; }
    @Override public synchronized void setMode(RunMode m) {
        this.mode = m;
        if (m == RunMode.STOP_AND_RESET_ENCODER) {
            encoderZeroTicks = shaftTicks();
            currentPositionTicks = 0;
            if (!externallyDriven) omegaRadS = 0;
            lastKnownTicks = 0;
            tickBuffer = new SensorRingBuffer<>(200);
        }
    }
    @Override public synchronized RunMode getMode() { return mode; }
    @Override public synchronized void setZeroPowerBehavior(ZeroPowerBehavior b) { this.zeroPowerBehavior = b; }
    @Override public synchronized ZeroPowerBehavior getZeroPowerBehavior() { return zeroPowerBehavior; }
    @Override public synchronized int getCurrentPosition() { return lastKnownTicks; } // per R4: reads latency-delayed value, not the true instantaneous one
    @Override public synchronized void setTargetPosition(int position) { this.targetPosition = position; }
    @Override public synchronized int getTargetPosition() { return targetPosition; }
    @Override public synchronized boolean isBusy() {
        return mode == RunMode.RUN_TO_POSITION && Math.abs(getCurrentPosition() - targetPosition) > 5;
    }

    @Override public synchronized void setVelocity(double angularRate) {
        this.targetVelocityFraction = angularRate / spec.omegaNoLoadRadS;
    }
    @Override public synchronized double getVelocity() { return (omegaRadS / (2 * Math.PI)) * spec.encoderCountsPerRev
        * (direction == Direction.REVERSE ? -1 : 1); }
    @Override public synchronized void setPIDFCoefficients(RunMode m, PIDFCoefficients p) { pidf.put(m, p); }
    @Override public synchronized PIDFCoefficients getPIDFCoefficients(RunMode m) { return pidf.get(m); }
    @Override public synchronized double getCurrent(CurrentUnit unit) { return unit.fromAmps(Math.abs(lastCurrentAmps)); }
    @Override public synchronized void setCurrentAlert(double current, CurrentUnit unit) { }

    @Override public synchronized String getDeviceName() { return name; }
    @Override public synchronized String getConnectionInfo() { return "Simulated motor \"" + name + "\" (sku=" + spec.sku + ", ratio=" + spec.ratio + ")"; }
    @Override public synchronized void close() { }

    public synchronized MotorSpec getSpec() { return spec; }
    public synchronized double getPhysicalShaftRadians() { return externalShaftRadians; }
    public synchronized double getOmegaRadS() { return omegaRadS; }
    public synchronized double getRawTicks() { return currentPositionTicks; }
}
