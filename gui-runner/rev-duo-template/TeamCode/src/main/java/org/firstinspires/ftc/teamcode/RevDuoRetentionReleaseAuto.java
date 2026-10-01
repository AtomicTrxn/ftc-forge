package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Adds deliberate reverse release to the capture/carry demonstration. */
@Autonomous(name="RevDuoRetentionReleaseAuto")
public class RevDuoRetentionReleaseAuto extends RevDuoRetentionAuto {
    @Override protected boolean releaseAfterCarry(){return true;}
}
