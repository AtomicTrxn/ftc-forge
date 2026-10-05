package simrunner;
import com.jme3.math.*;
import simcore.SimIMU;
/** URDF x forward/y left/z up converted from jME x/-z/y; intrinsic ZYX. */
final class RobotOrientation {
    static void update(SimIMU imu,Quaternion pose,Vector3f angularWorld,long timeMs) {
        var forward=pose.mult(Vector3f.UNIT_X);var left=pose.mult(Vector3f.UNIT_Z.negate());var up=pose.mult(Vector3f.UNIT_Y);
        double yaw=Math.atan2(-forward.z,forward.x),pitch=Math.atan2(-forward.y,Math.hypot(forward.x,forward.z)),roll=Math.atan2(left.y,up.y);
        var rate=pose.inverse().mult(angularWorld);
        imu.update(yaw,pitch,roll,rate.x,-rate.z,rate.y,timeMs);
    }
}
