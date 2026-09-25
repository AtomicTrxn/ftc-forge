package com.qualcomm.hardware.rev;

/**
 * Mock of com.qualcomm.hardware.rev.RevHubOrientationOnRobot -- required to construct
 * IMU.Parameters (R1's amendment: this class is referenced by nearly every OpMode that
 * initializes the IMU, even though it wasn't in the plan's original stub list).
 */
public class RevHubOrientationOnRobot {

    public enum LogoFacingDirection { UP, DOWN, FORWARD, BACKWARD, LEFT, RIGHT }
    public enum UsbFacingDirection { UP, DOWN, FORWARD, BACKWARD, LEFT, RIGHT }

    private final LogoFacingDirection logoFacingDirection;
    private final UsbFacingDirection usbFacingDirection;

    public RevHubOrientationOnRobot(LogoFacingDirection logo, UsbFacingDirection usb) {
        this.logoFacingDirection = logo;
        this.usbFacingDirection = usb;
    }

    public LogoFacingDirection getLogoFacingDirection() { return logoFacingDirection; }
    public UsbFacingDirection getUsbFacingDirection() { return usbFacingDirection; }
}
