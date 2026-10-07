package simcore;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import physics.MotorModel;
import physics.MotorSpec;
import physics.InterpolatedSensorBuffer;

import java.util.EnumMap;
import java.util.Map;

/**
 * Backing implementation for DcMotorEx, now wired to R4's corrected motor model (Phase 3) --
 * replacing Phase 1's placeholder linear tick integration.
 *
 * Unbound motors use a placeholder rotor/load inertia. Imported articulated mechanisms
 * instead receive actual shaft motion from Bullet and supply electrical torque to the joint.
 * Encoder position and velocity are read through an InterpolatedSensorBuffer to model I2C read latency (R4's
 * reassessment: cheap enough to ship in Phase 3 rather than deferring further).
 */
public class SimDcMotorEx implements DcMotorEx {

    public static final double ROTATIONAL_INERTIA_KG_M2 = 0.0015; // placeholder default; see class javadoc and MotorTuning
    public static final long DEFAULT_ENCODER_LATENCY_MS = 8; // per R4's 7-10ms I2C default
    private long encoderLatencyMs = DEFAULT_ENCODER_LATENCY_MS;

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
    private double rotorInertiaKgM2 = ROTATIONAL_INERTIA_KG_M2;
    private boolean powerCut;
    private SimVoltageSensor hub;
    private long currentLatencyMs;
    private InterpolatedSensorBuffer currentBuffer = new InterpolatedSensorBuffer(1000);
    private double lastKnownCurrentAmps;

    private final Map<RunMode, PIDFCoefficients> pidf = new EnumMap<>(RunMode.class);
    // Position and velocity come from one simulated bulk read, so they share one latency.
    private InterpolatedSensorBuffer tickBuffer = new InterpolatedSensorBuffer(200);
    private InterpolatedSensorBuffer velocityBuffer = new InterpolatedSensorBuffer(200);
    private int lastKnownTicks = 0;
    private double lastKnownVelocityTicksPerS = 0;

    public SimDcMotorEx(String name, MotorSpec spec) {
        this.name = name;
        this.spec = spec;
        pidf.put(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1.0, 0, 0, 0));
        pidf.put(RunMode.RUN_TO_POSITION, new PIDFCoefficients(0.01, 0, 0, 0));
    }

    /** Pure function of current state -- safe to call twice per tick (see HardwareMapBuilder). */
    public synchronized double commandedPower() {
        if (powerCut) return 0;
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

    public synchronized void setEncoderLatencyMs(long ms) {
        if (ms < 0) throw new IllegalArgumentException("Encoder latency must be nonnegative");
        this.encoderLatencyMs = ms;
    }

    public synchronized long getEncoderLatencyMs() { return encoderLatencyMs; }

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

        // A floating motor (FLOAT at zero power, or the driver cut off in a brownout) has no electrical
        // torque, so only friction slows it; BRAKE shorts the windings and adds back-EMF braking.
        boolean floating = signedPower == 0 && (zeroPowerBehavior == ZeroPowerBehavior.FLOAT || powerCut);
        double tau = floating ? -motorModel.frictionTorque(omegaRadS) : motorModel.torque(spec, omegaRadS, vActual);
        double alpha = tau / rotorInertiaKgM2;
        if (!externallyDriven) omegaRadS += alpha * dtSeconds;

        lastCurrentAmps = floating || (externallyDriven && mode == RunMode.STOP_AND_RESET_ENCODER)
            ? 0 : motorModel.current(spec, omegaRadS, vActual);
        motorModel.tickThermal(spec, lastCurrentAmps, dtSeconds);

        if (!externallyDriven) {
            externalShaftRadians += omegaRadS * dtSeconds;
            currentPositionTicks = shaftTicks() - encoderZeroTicks;
        }

        currentBuffer.push(simTimeMs, Math.abs(lastCurrentAmps));
        double delayedCurrent = currentLatencyMs == 0 ? Double.NaN : currentBuffer.read(simTimeMs, currentLatencyMs);
        lastKnownCurrentAmps = Double.isNaN(delayedCurrent) ? Math.abs(lastCurrentAmps) : delayedCurrent;
        tickBuffer.push(simTimeMs, currentPositionTicks);
        velocityBuffer.push(simTimeMs, trueVelocityTicksPerS());
        double delayedTicks = tickBuffer.read(simTimeMs, encoderLatencyMs);
        if (!Double.isNaN(delayedTicks)) lastKnownTicks = (int) Math.round(delayedTicks);
        double delayedVelocity = velocityBuffer.read(simTimeMs, encoderLatencyMs);
        if (!Double.isNaN(delayedVelocity)) lastKnownVelocityTicksPerS = delayedVelocity;
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
        if (command == 0 && (zeroPowerBehavior == ZeroPowerBehavior.FLOAT || powerCut)) return 0;
        double voltage = command * batteryVoltage * (direction == Direction.FORWARD ? 1 : -1);
        return motorModel.torque(spec, omegaRadS, voltage);
    }

    public synchronized double externalTorqueDamping() {
        if (mode == RunMode.STOP_AND_RESET_ENCODER ||
            (commandedPower() == 0 && (zeroPowerBehavior == ZeroPowerBehavior.FLOAT || powerCut))) return 0;
        return spec.tauStallNm / spec.omegaNoLoadRadS;
    }

    private static double clamp(double v) { return Math.max(-1.0, Math.min(1.0, v)); }

    @Override public synchronized void setDirection(Direction d) {
        if (direction != d) {
            encoderZeroTicks = -encoderZeroTicks;
            currentPositionTicks = -currentPositionTicks;
            lastKnownTicks = -lastKnownTicks;
            resetEncoderHistory();
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
            resetEncoderHistory();
        }
    }
    @Override public synchronized RunMode getMode() { return mode; }
    @Override public synchronized void setZeroPowerBehavior(ZeroPowerBehavior b) { this.zeroPowerBehavior = b; }
    @Override public synchronized ZeroPowerBehavior getZeroPowerBehavior() { return zeroPowerBehavior; }
    /** Latency-delayed encoder state as of now; what a bus read returns. */
    public record Reading(int ticks, double velocityTicksPerS) { }

    synchronized Reading liveReading() { return new Reading(lastKnownTicks, lastKnownVelocityTicksPerS); }

    public synchronized void attachHub(SimVoltageSensor hub) { this.hub = hub; hub.attach(this); }

    /** Battery brownout: the driver is off, so the motor floats with no electrical torque or current. */
    public synchronized void setPowerCut(boolean cut) { this.powerCut = cut; }
    public synchronized boolean isPowerCut() { return powerCut; }

    public synchronized void setCurrentLatencyMs(long ms) {
        if (ms < 0) throw new IllegalArgumentException("Current latency must be nonnegative");
        currentLatencyMs = ms;
    }

    /** Applies configured overrides; null fields keep the current value. */
    public synchronized void applyTuning(MotorTuning t) {
        if (t.rotorInertiaKgM2() != null) rotorInertiaKgM2 = t.rotorInertiaKgM2();
        if (t.staticFrictionNm() != null || t.viscousFrictionNmSPerRad() != null)
            configureFriction(t.staticFrictionNm() != null ? t.staticFrictionNm() : motorModel.tauStaticNm,
                t.viscousFrictionNmSPerRad() != null ? t.viscousFrictionNmSPerRad() : motorModel.viscousBNms);
        if (t.thermalThresholdFraction() != null) motorModel.thermalThresholdAmps = t.thermalThresholdFraction();
        if (t.thermalTimeConstantS() != null) motorModel.thermalTimeConstantS = t.thermalTimeConstantS();
        if (t.thermalMaxDerate() != null) motorModel.thermalMaxDerate = t.thermalMaxDerate();
        if (t.velocityPGain() != null) {
            var c = pidf.get(RunMode.RUN_USING_ENCODER);
            pidf.put(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(t.velocityPGain(), c.i, c.d, c.f));
        }
        if (t.positionPGain() != null) {
            var c = pidf.get(RunMode.RUN_TO_POSITION);
            pidf.put(RunMode.RUN_TO_POSITION, new PIDFCoefficients(t.positionPGain(), c.i, c.d, c.f));
        }
    }

    public synchronized double getRotorInertiaKgM2() { return rotorInertiaKgM2; }

    // Not synchronized on the motor: the hub takes its own lock first and then reads this motor's state.
    @Override public int getCurrentPosition() { return hub == null ? liveReading().ticks() : hub.read(this, SimVoltageSensor.Kind.POSITION).ticks(); }
    @Override public synchronized void setTargetPosition(int position) { this.targetPosition = position; }
    @Override public synchronized int getTargetPosition() { return targetPosition; }
    @Override public boolean isBusy() {
        return getMode() == RunMode.RUN_TO_POSITION && Math.abs(getCurrentPosition() - getTargetPosition()) > 5;
    }

    @Override public synchronized void setVelocity(double angularRate) {
        this.targetVelocityFraction = angularRate / spec.omegaNoLoadRadS;
    }
    private double trueVelocityTicksPerS() {
        return (omegaRadS / (2 * Math.PI)) * spec.encoderCountsPerRev * (direction == Direction.REVERSE ? -1 : 1);
    }

    private void resetEncoderHistory() {
        tickBuffer = new InterpolatedSensorBuffer(200);
        velocityBuffer = new InterpolatedSensorBuffer(200);
        lastKnownVelocityTicksPerS = 0;
    }

    /** Like the hub's bulk read, velocity is as old as the encoder position, not the instantaneous value. */
    @Override public double getVelocity() { return hub == null ? liveReading().velocityTicksPerS() : hub.read(this, SimVoltageSensor.Kind.VELOCITY).velocityTicksPerS(); }
    @Override public synchronized void setPIDFCoefficients(RunMode m, PIDFCoefficients p) { pidf.put(m, p); }
    @Override public synchronized PIDFCoefficients getPIDFCoefficients(RunMode m) { return pidf.get(m); }
    @Override public synchronized double getCurrent(CurrentUnit unit) { return unit.fromAmps(lastKnownCurrentAmps); }
    @Override public synchronized void setCurrentAlert(double current, CurrentUnit unit) { }

    @Override public synchronized String getDeviceName() { return name; }
    @Override public synchronized String getConnectionInfo() { return "Simulated motor \"" + name + "\" (sku=" + spec.sku + ", ratio=" + spec.ratio + ")"; }
    @Override public synchronized void close() { }

    public synchronized MotorSpec getSpec() { return spec; }
    public synchronized double getPhysicalShaftRadians() { return externalShaftRadians; }
    public synchronized double getOmegaRadS() { return omegaRadS; }
    public synchronized double getRawTicks() { return currentPositionTicks; }
}
