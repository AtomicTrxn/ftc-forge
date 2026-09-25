package com.qualcomm.robotcore.hardware;

/** Plain data holder -- per R1, trivial to stub. Simulated/manual input writes these fields each tick. */
public class Gamepad {
    public double left_stick_x, left_stick_y, right_stick_x, right_stick_y;
    public double left_trigger, right_trigger;
    public boolean a, b, x, y;
    public boolean dpad_up, dpad_down, dpad_left, dpad_right;
    public boolean left_bumper, right_bumper;
    public boolean left_stick_button, right_stick_button;
    public boolean start, back, guide;

    public void rumble(int durationMs) { /* no-op in v1 */ }
    public void setLedColor(double r, double g, double b, int durationMs) { /* no-op in v1 */ }
}
