package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Mock of com.qualcomm.robotcore.hardware.DcMotorEx.
 * The velocity/PIDF/current surface here is only as good as Phase 3's motor model (R4) --
 * this mock's default implementation (see simcore.SimDcMotorEx) uses a placeholder
 * integration, not R4's corrected torque/current equations, and says so explicitly.
 */
public interface DcMotorEx extends DcMotor {
    void setVelocity(double angularRate);
    double getVelocity();
    void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidf);
    PIDFCoefficients getPIDFCoefficients(RunMode mode);
    double getCurrent(CurrentUnit unit);
    void setCurrentAlert(double current, CurrentUnit unit);
}
