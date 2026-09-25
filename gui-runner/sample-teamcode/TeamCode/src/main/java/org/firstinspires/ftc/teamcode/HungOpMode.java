package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Deliberately buggy sample: a real, common team mistake -- a while(true) loop that never
 * checks opModeIsActive(). Used only to validate Phase 1's Definition of Done: "a
 * deliberately-hung OpMode is confirmed not to wedge the whole executor (watchdog isolates it)."
 * Never run this by accident in a real session; it exists purely for that one test.
 */
@Autonomous(name = "Deliberately Hung OpMode (watchdog test only)")
public class HungOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (true) {
            // Deliberately does not check opModeIsActive() -- this is the bug under test.
        }
    }
}
