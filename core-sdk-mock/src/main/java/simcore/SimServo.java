package simcore;

import com.qualcomm.robotcore.hardware.Servo;

public class SimServo implements Servo, ServoLoad {
    private final String name;
    private Direction direction = Direction.FORWARD;
    private volatile double position = 0.5;
    private volatile boolean commanded;
    private double lastSeenPosition = 0.5;
    private long activeUntilMs = -1;

    public SimServo(String name) { this.name = name; }

    @Override public void setDirection(Direction d) { this.direction = d; }
    @Override public Direction getDirection() { return direction; }
    @Override public void setPosition(double position) {
        this.position = Math.max(0, Math.min(1, position));
        commanded = true;
    }

    @Override public synchronized double drawAmps(long timeMs, double holdA, double activeA, long activeMs) {
        if (!commanded) return 0;
        if (position != lastSeenPosition) {
            lastSeenPosition = position;
            activeUntilMs = timeMs + activeMs;
        }
        return timeMs < activeUntilMs ? activeA : holdA;
    }
    @Override public double getPosition() { return position; }

    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated servo \"" + name + "\""; }
    @Override public void close() { }
}
