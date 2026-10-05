package org.firstinspires.ftc.vision.apriltag;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public class AprilTagMetadata {
    public final int id;
    public final String name;
    public final double tagsize;
    public final DistanceUnit distanceUnit;
    public AprilTagMetadata(int id,String name,double tagsize,DistanceUnit distanceUnit){this.id=id;this.name=name;this.tagsize=tagsize;this.distanceUnit=distanceUnit;}
}
