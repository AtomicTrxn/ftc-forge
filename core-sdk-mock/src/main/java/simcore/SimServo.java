package simcore;

import com.qualcomm.robotcore.hardware.Servo;

public class SimServo implements Servo {
    private final String name;
    private Direction direction = Direction.FORWARD;
    private double position = 0.5;

    public SimServo(String name) { this.name = name; }

    @Override public void setDirection(Direction d) { this.direction = d; }
    @Override public Direction getDirection() { return direction; }
    @Override public void setPosition(double position) { this.position = Math.max(0, Math.min(1, position)); }
    @Override public double getPosition() { return position; }

    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated servo \"" + name + "\""; }
    @Override public void close() { }
}
