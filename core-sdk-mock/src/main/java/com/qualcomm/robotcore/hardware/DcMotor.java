package com.qualcomm.robotcore.hardware;

/**
 * Mock of com.qualcomm.robotcore.hardware.DcMotor.
 * RunMode/ZeroPowerBehavior semantics are real behavioral state, not accept-and-ignore
 * (per R1's stub-complexity note) -- the actual torque/speed math those modes drive
 * belongs to Phase 3's motor model (R4); this mock only owns the mode state machine.
 */
public interface DcMotor extends DcMotorSimple {

    enum RunMode {
        RUN_WITHOUT_ENCODER, RUN_USING_ENCODER, RUN_TO_POSITION, STOP_AND_RESET_ENCODER
    }

    enum ZeroPowerBehavior {
        UNKNOWN, BRAKE, FLOAT
    }

    void setMode(RunMode mode);
    RunMode getMode();
    void setZeroPowerBehavior(ZeroPowerBehavior behavior);
    ZeroPowerBehavior getZeroPowerBehavior();
    int getCurrentPosition();
    void setTargetPosition(int position);
    int getTargetPosition();
    boolean isBusy();
}
