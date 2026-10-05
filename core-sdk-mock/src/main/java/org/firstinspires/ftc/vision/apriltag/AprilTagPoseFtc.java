package org.firstinspires.ftc.vision.apriltag;
/** Camera x right, y forward, z up; default processor output inches/degrees. */
public class AprilTagPoseFtc {
    public double x,y,z,yaw,pitch,roll,range,bearing,elevation;
    public AprilTagPoseFtc() { }
    public AprilTagPoseFtc(double x,double y,double z,double yaw,double pitch,double roll,double range,double bearing,double elevation) {
        this.x=x;this.y=y;this.z=z;this.yaw=yaw;this.pitch=pitch;this.roll=roll;this.range=range;this.bearing=bearing;this.elevation=elevation;
    }
}
