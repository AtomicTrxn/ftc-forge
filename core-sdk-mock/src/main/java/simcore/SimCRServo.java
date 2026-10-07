package simcore;

import com.qualcomm.robotcore.hardware.CRServo;

public class SimCRServo implements CRServo, ServoLoad {
    private final String name;
    private Direction direction = Direction.FORWARD;
    private volatile double power = 0;

    public SimCRServo(String name) { this.name = name; }

    @Override public void setDirection(Direction d) { this.direction = d; }
    @Override public Direction getDirection() { return direction; }
    @Override public void setPower(double p) { this.power = Math.max(-1, Math.min(1, p)); }
    @Override public double getPower() { return power; }

    @Override public double drawAmps(long timeMs, double holdA, double activeA, long activeMs) {
        return power == 0 ? 0 : activeA * Math.abs(power);
    }

    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated CRServo \"" + name + "\""; }
    @Override public void close() { }
}
