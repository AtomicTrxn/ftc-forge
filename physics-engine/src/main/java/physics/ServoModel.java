package physics;

/** Electrical-speed envelope and positional feedback for a rotational servo output shaft. */
public final class ServoModel {
    public record Spec(double stallTorqueNm, double noLoadSpeedRadS, double travelRad,
                       double positionGainPerS, double velocityGainNmPerRadS, double deadbandRad) {
        public Spec {
            if (!Double.isFinite(stallTorqueNm) || stallTorqueNm <= 0
                || !Double.isFinite(noLoadSpeedRadS) || noLoadSpeedRadS <= 0
                || !Double.isFinite(travelRad) || travelRad <= 0
                || !Double.isFinite(positionGainPerS) || positionGainPerS <= 0
                || !Double.isFinite(velocityGainNmPerRadS) || velocityGainNmPerRadS <= 0
                || !Double.isFinite(deadbandRad) || deadbandRad < 0)
                throw new IllegalArgumentException("Servo physics parameters must be finite and positive (deadband may be zero)");
        }
    }

    public static double effort(Spec spec, double shaftPositionErrorRad, double shaftVelocityRadS) {
        double targetVelocity = Math.abs(shaftPositionErrorRad) <= spec.deadbandRad ? 0
            : clamp(shaftPositionErrorRad * spec.positionGainPerS,
                -spec.noLoadSpeedRadS, spec.noLoadSpeedRadS);
        double demand = spec.velocityGainNmPerRadS * (targetVelocity - shaftVelocityRadS);
        double movingWithEffort = Math.max(0, shaftVelocityRadS * Math.signum(demand));
        double available = spec.stallTorqueNm * Math.max(0, 1 - movingWithEffort / spec.noLoadSpeedRadS);
        return clamp(demand, -available, available);
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }
}
