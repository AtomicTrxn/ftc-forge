package simrunner;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import java.nio.file.*;
import java.util.*;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.math.*;
import simcore.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.apriltag.*;
import static org.junit.jupiter.api.Assertions.*;
class SceneSensorsTest {
    @TempDir Path tmp;
    @BeforeAll static void natives(){com.jme3.system.NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    private RobotMotionDemo demo()throws Exception{var c=SyntheticRobots.importRobot(tmp,true,true,false,SyntheticRobots.Dimensions.standard());return new RobotMotionDemo(c.motionSetup(),new DesktopAssetManager(true));}
    private void ticks(RobotMotionDemo d,int count){for(int i=0;i<count;i++)d.space.update(1f/120,0);}
    private Map<String,Object> sensor(String name,String type){return new LinkedHashMap<>(Map.of("name",name,"type",type,"xyz_m",List.of(.2,0,0),"update_hz",120.,"max_range_m",2.));}
    private PhysicsRigidBody wall(RobotMotionDemo d,float x){var body=new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(.02f,1,1)),0);body.setPhysicsLocation(new Vector3f(x,.5f,0));d.space.add(body);return body;}
    @Test void distanceColorLatencyPoseAndNoHitFollowNativeScene()throws Exception {
        try(var d=demo()){ticks(d,100);var distance=new SimDistanceSensor("range");var color=new SimColorSensor("color");d.setup.hardware().register("range",distance);d.setup.hardware().register("color",color);wall(d,1);
            var range=sensor("range","distance");range.put("latency_ms",50.);var paint=Map.of("min_xyz_m",List.of(.9,-1,-1),"max_xyz_m",List.of(1.1,1,2),"rgba",List.of(220,10,20,255));
            try(var sensors=new SceneSensors(d.world,d.robot,d.setup.hardware(),SceneSensorConfig.parse(List.of(range,sensor("color","color"))),Map.of("colors",List.of(paint)))) {
                ticks(d,4);assertEquals(Double.POSITIVE_INFINITY,distance.getDistance(DistanceUnit.METER));assertEquals(220,color.red());assertEquals(10,color.green());assertEquals(255,color.alpha());
                ticks(d,4);assertEquals(.78,distance.getDistance(DistanceUnit.METER),.002);assertEquals(780,distance.getDistance(DistanceUnit.MM),2);
                d.space.setGravity(Vector3f.ZERO);for(var b:d.space.getRigidBodyList())if(b.isDynamic())b.setPhysicsLocation(b.getPhysicsLocation().add(0,0,3));ticks(d,10);assertEquals(Double.POSITIVE_INFINITY,distance.getDistance(DistanceUnit.METER));assertEquals(0,color.alpha());
                assertEquals(2,sensors.summaries().size());
            }
        }
    }
    @Test void mechanismMountTracksPhysicalLinkAndTouchRequiresLocalNativeContact()throws Exception {
        try(var d=demo()){ticks(d,100);var distance=new SimDistanceSensor("range");var touch=new SimTouchSensor("switch");d.setup.hardware().register("range",distance);d.setup.hardware().register("switch",touch);
            var range=sensor("range","distance");range.put("link","slide");range.put("xyz_m",List.of(0,0,0));var switchSpec=sensor("switch","touch");switchSpec.put("xyz_m",List.of(.15,0,0));switchSpec.put("contact_radius_m",.16);
            var obstacle=wall(d,1);
            try(var sensors=new SceneSensors(d.world,d.robot,d.setup.hardware(),SceneSensorConfig.parse(List.of(range,switchSpec)),Map.of())) {
                ticks(d,2);double before=distance.getDistance(DistanceUnit.METER);d.setup.hardware().get(SimDcMotorEx.class,"slideMotor").setPower(.2);ticks(d,40);double after=distance.getDistance(DistanceUnit.METER);assertTrue(after<before-.015);
                assertFalse(touch.isPressed());d.space.remove(obstacle);var bumper=wall(d,.15f);ticks(d,1);assertTrue(touch.isPressed());d.space.remove(bumper);ticks(d,3);assertFalse(touch.isPressed());
            }
        }
    }
    @Test void geometricTagsHaveSdkUnitsFrustumOcclusionAndPortalLifecycle()throws Exception {
        try(var d=demo()){ticks(d,100);var camera=new WebcamName("camera");d.setup.hardware().register("camera",camera);var spec=sensor("camera","camera");spec.put("xyz_m",List.of(.2,0,.2));spec.put("max_range_m",4.);
            var origin=d.world.robotPointWorld(new Vector3f(.2f,.2f,0));var tagAt=origin.add(1,.2f,-.2f);
            var tag=Map.of("id",7,"name","test tag","size_m",.16,"xyz_m",List.of(tagAt.x,-tagAt.z,tagAt.y),"rpy_rad",List.of(0,0,Math.PI));
            var processor=new AprilTagProcessor.Builder().build();var metric=new AprilTagProcessor.Builder().setOutputUnits(DistanceUnit.METER,AngleUnit.RADIANS).build();var portal=new VisionPortal.Builder().setCamera(camera).addProcessor(processor).addProcessor(metric).build();assertEquals(VisionPortal.CameraState.CAMERA_DEVICE_CLOSED,portal.getCameraState());
            try(var sensors=new SceneSensors(d.world,d.robot,d.setup.hardware(),SceneSensorConfig.parse(List.of(spec)),Map.of("tags",List.of(tag)))) {
                ticks(d,2);assertEquals(VisionPortal.CameraState.STREAMING,portal.getCameraState());assertEquals(1,processor.getDetections().size());var detection=processor.getDetections().get(0);assertEquals(7,detection.id);assertEquals("test tag",detection.metadata.name);
                assertEquals(-.2/ .0254,detection.ftcPose.x,.02);assertEquals(1/.0254,detection.ftcPose.y,.02);assertEquals(.2/.0254,detection.ftcPose.z,.02);assertEquals(Math.toDegrees(Math.atan2(.2,1)),detection.ftcPose.bearing,.05);assertEquals(0,detection.ftcPose.yaw,.05);assertTrue(detection.frameAcquisitionNanoTime>0);assertEquals(1,metric.getDetections().get(0).ftcPose.y,.002);
                detection.ftcPose.x=99;assertNotEquals(99,processor.getDetections().get(0).ftcPose.x);
                portal.setProcessorEnabled(processor,false);assertTrue(processor.getDetections().isEmpty());portal.setProcessorEnabled(processor,true);portal.stopStreaming();assertEquals(VisionPortal.CameraState.CAMERA_DEVICE_READY,portal.getCameraState());assertTrue(processor.getDetections().isEmpty());portal.resumeStreaming();
                var obstruction=wall(d,origin.x+.5f);ticks(d,2);assertTrue(processor.getDetections().isEmpty());d.space.remove(obstruction);ticks(d,2);assertEquals(1,processor.getDetections().size());
                // Same camera yawed away: all corners must fit the visible camera frustum.
                for(var b:d.space.getRigidBodyList())if(b.isDynamic()){b.setPhysicsRotation(new Quaternion().fromAngleAxis((float)(Math.PI/2),Vector3f.UNIT_Y));}d.space.setGravity(Vector3f.ZERO);ticks(d,2);assertTrue(processor.getDetections().isEmpty());portal.close();assertEquals(VisionPortal.CameraState.CAMERA_DEVICE_CLOSED,portal.getCameraState());
            }
        }
    }
    @Test void unknownBindingsAndInvalidSettingsFailBeforeStartingSimulation()throws Exception {
        assertThrows(IllegalArgumentException.class,()->SceneSensorConfig.parse(List.of(Map.of("name","range","type","distance","max_range_m",-1))));
        assertThrows(IllegalArgumentException.class,()->SceneSensorConfig.parse(List.of(sensor("range","distance"),sensor("range","distance"))));
        try(var d=demo()){assertThrows(IllegalArgumentException.class,()->new SceneSensors(d.world,d.robot,d.setup.hardware(),SceneSensorConfig.parse(List.of(sensor("unknown","distance"))),Map.of()));}
    }
}
