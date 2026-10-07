package physics;

/**
 * Slow battery state on top of {@link BatteryModel}'s per-tick sag: state of charge (open-circuit
 * voltage falls as charge is drawn) and a hub brownout latch. Every parameter is a public field so
 * it can be set from configuration and fitted from telemetry later. Both effects are off by default
 * ({@code capacityAh == 0}, {@code brownoutV == 0}); the shape parameters are placeholders, not
 * measured NiMH curves.
 */
public class BatteryPack {
    /** Total capacity in amp-hours; 0 disables state-of-charge tracking (open-circuit voltage stays constant). */
    public double capacityAh = 0;
    /** Open-circuit voltage at zero charge. The full-charge value is supplied by the caller each tick. */
    public double emptyVoltageV = 10.5;
    /** Curve shape: Voc = empty + (full - empty) * charge^shape. 1 is linear; above 1 the voltage holds up longer then collapses. */
    public double shape = 1.0;
    /** Terminal voltage below which the hub browns out; 0 disables brownout. */
    public double brownoutV = 0;
    /** Minimum time motors stay cut after a brownout (hub reboot time). */
    public double brownoutHoldS = 2.0;
    /** Open-circuit voltage the pack must recover to before motors return after the hold. */
    public double brownoutRecoveryV = 0;

    /**
     * Internal resistance when the pack is empty; the resistance rises linearly from the base value
     * at full charge. Negative means "same as the base value" (no rise). Only used with charge tracking.
     */
    public double emptyResistanceOhm = -1;

    private double charge = 1.0;
    private boolean brownedOut;
    private double brownoutStartS;

    public synchronized void setChargeFraction(double fraction) {
        if (!(fraction >= 0 && fraction <= 1)) throw new IllegalArgumentException("Charge fraction must be 0..1");
        charge = fraction;
    }

    public synchronized double chargeFraction() { return charge; }
    public synchronized boolean isBrownedOut() { return brownedOut; }

    /** Open-circuit voltage at the current charge, or {@code fullV} when charge tracking is off. */
    public synchronized double openCircuitV(double fullV) {
        if (capacityAh <= 0) return fullV;
        double empty = Math.min(emptyVoltageV, fullV);
        return empty + (fullV - empty) * Math.pow(charge, shape);
    }

    /** Internal resistance at the current charge: {@code baseOhm} when tracking is off or no empty value is set. */
    public synchronized double resistanceOhm(double baseOhm) {
        if (capacityAh <= 0 || emptyResistanceOhm < 0) return baseOhm;
        return baseOhm + (emptyResistanceOhm - baseOhm) * (1 - charge);
    }

    /** Draws {@code currentA} (battery side) for {@code dtS}. */
    public synchronized void consume(double currentA, double dtS) {
        if (capacityAh <= 0 || currentA <= 0) return;
        charge = Math.max(0, charge - currentA * dtS / (capacityAh * 3600));
    }

    /** Updates the brownout latch from this tick's terminal and open-circuit voltage. */
    public synchronized void updateBrownout(double terminalV, double openCircuitV, double timeS) {
        if (brownoutV <= 0) { brownedOut = false; return; }
        if (!brownedOut && terminalV < brownoutV) {
            brownedOut = true;
            brownoutStartS = timeS;
        } else if (brownedOut && timeS - brownoutStartS >= brownoutHoldS
            && openCircuitV >= Math.max(brownoutRecoveryV, brownoutV)) {
            brownedOut = false;
        }
    }
}
