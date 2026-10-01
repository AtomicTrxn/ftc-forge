package simrunner;

import com.jme3.math.*;
import com.jme3.bounding.BoundingBox;
import java.util.*;

/** Layout checks shared by guided preflight and actual simulator startup. */
final class SceneChecks {
    static void robotStart(ImportedRobotScene robot, Vector3f start, float yawRad, Vector3f half, float floorTop, boolean biobuzz) {
        float minX=-.2286f,maxX=.2286f,minZ=-.2286f,maxZ=.2286f,minY=-.1f;
        if(robot!=null){robot.root.updateGeometricState();if(robot.root.getWorldBound() instanceof BoundingBox box){
            minX=Float.POSITIVE_INFINITY;maxX=Float.NEGATIVE_INFINITY;minZ=Float.POSITIVE_INFINITY;maxZ=Float.NEGATIVE_INFINITY;minY=box.getCenter().y-box.getYExtent();
            var yaw=new Quaternion().fromAngleAxis(yawRad,Vector3f.UNIT_Y);
            for(float x:new float[]{box.getCenter().x-box.getXExtent(),box.getCenter().x+box.getXExtent()})for(float z:new float[]{box.getCenter().z-box.getZExtent(),box.getCenter().z+box.getZExtent()}){var p=yaw.mult(new Vector3f(x,0,z));minX=Math.min(minX,p.x);maxX=Math.max(maxX,p.x);minZ=Math.min(minZ,p.z);maxZ=Math.max(maxZ,p.z);}
        }}
        if(!Float.isFinite(yawRad)||!Vector3f.isValidVector(start)||start.x+minX<-half.x||start.x+maxX>half.x||start.z+minZ<-half.z||start.z+maxZ>half.z||start.y+minY<floorTop-.004f)
            throw new IllegalArgumentException("Robot starts below the floor or outside the field. Edit its starting position.");
        if(biobuzz&&start.x+maxX>-.65f&&start.x+minX<.65f&&start.z+maxZ>-.5f&&start.z+minZ<.5f)throw new IllegalArgumentException("Robot overlaps the HIVE/frame area. Choose a starting position away from the center.");
    }
    static void contacts(PhysicsWorld world, Set<Long> environment, Vector3f half, float floorTop) {
        for(var body:world.space().getRigidBodyList())if(!body.isDynamic())environment.add(body.nativeId());
        for(var p:world.gamePieces())environment.add(p.body().nativeId());
        for(var piece:world.gamePieces()){
            var bounds=piece.body().boundingBox(null);var min=bounds.getMin(null);var max=bounds.getMax(null);
            if(!Vector3f.isValidVector(min)||!Vector3f.isValidVector(max)||min.y<floorTop-.004f||min.x<-half.x-.004f||max.x>half.x+.004f||min.z<-half.z-.004f||max.z>half.z+.004f)
                throw new IllegalArgumentException("Piece "+piece.id()+" starts outside the field or below its floor. Edit its placement.");
            world.space().contactTest(piece.body(),event->{if(event.getDistance1()<-.004f)throw new IllegalArgumentException("Piece "+piece.id()+" overlaps an obstacle or another piece. Edit its placement.");});
        }
        for(var body:world.space().getRigidBodyList())if(body.isDynamic()&&!environment.contains(body.nativeId()))world.space().contactTest(body,event->{
            long other=event.getObjectA().nativeId()==body.nativeId()?event.getObjectB().nativeId():event.getObjectA().nativeId();
            if(environment.contains(other)&&event.getDistance1()<-.004f)throw new IllegalArgumentException("Robot overlaps a field obstacle or game piece. Edit its starting position.");
        });
    }
}
