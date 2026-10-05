package simrunner;
import com.jme3.bullet.*;
import com.jme3.bullet.collision.*;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.math.*;
import com.jme3.scene.*;
import com.qualcomm.robotcore.hardware.*;
import simcore.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.*;
import java.util.*;

/** Native tick measurements, cached for asynchronous SDK readers. No render-frame clock. */
final class SceneSensors implements PhysicsTickListener,AutoCloseable {
    record Tag(int id,String name,double size,Transform pose) { }
    record ColorRegion(Vector3f min,Vector3f max,int[] rgba) { }
    record Pending(double available,double acquired,Object value) { }
    private final PhysicsWorld world;
    private final ArticulatedRobot robot;
    private final Set<Long> ownBodies;
    private final List<Binding> bindings=new ArrayList<>();
    private final List<Tag> tags=new ArrayList<>();
    private final List<ColorRegion> colors=new ArrayList<>();
    private double clock;
    private boolean closed;
    private final class Binding {
        final SceneSensorConfig spec;final HardwareDevice device;final Random random;final ArrayDeque<Pending> queue=new ArrayDeque<>();double next;
        Binding(SceneSensorConfig spec,HardwareDevice device){this.spec=spec;this.device=device;random=new Random(spec.seed()+spec.name().hashCode());}
    }
    SceneSensors(PhysicsWorld world,ArticulatedRobot robot,HardwareMap hardware,List<SceneSensorConfig> specs,Map<String,Object> field) {
        this.world=world;this.robot=robot;ownBodies=robot==null?new HashSet<>(Set.of(world.chassisBody().nativeId())):new HashSet<>(robot.diagnosticBodies().keySet());
        if(world.flexibleIntake()!=null)for(var flap:world.flexibleIntake().flaps)for(var segment:flap.segments)ownBodies.add(segment.body().nativeId());
        for(var spec:specs) {
            HardwareDevice device=switch(spec.type()){case "distance"->hardware.get(SimDistanceSensor.class,spec.name());case "color"->hardware.get(SimColorSensor.class,spec.name());case "touch"->hardware.get(SimTouchSensor.class,spec.name());default->hardware.get(WebcamName.class,spec.name());};
            if(!spec.link().isEmpty()&&(robot==null||!robotHasLink(spec.link())))throw new IllegalArgumentException("Unknown sensor mount link: "+spec.link());
            if(device instanceof SimDistanceSensor distance)distance.setDistanceMeters(Double.POSITIVE_INFINITY);
            if(device instanceof WebcamName camera)camera.setSimulatedDetections(List.of());bindings.add(new Binding(spec,device));
        }
        var ids=new HashSet<Integer>();var rawTags=FieldPackage.maps(field.getOrDefault("tags",List.of()));if(rawTags.size()>64)throw new IllegalArgumentException("At most 64 field tags");
        for(var m:rawTags){double id=FieldPackage.num(m,"id");if(id!=(int)id||id<0||id>10000||!ids.add((int)id))throw new IllegalArgumentException("Tag IDs must be unique integers 0..10000");tags.add(new Tag((int)id,FieldPackage.str(m,"name"),SceneSensorConfig.n(m,"size_m",.16,.001,2),FieldPackage.transform(m)));}
        var rawColors=FieldPackage.maps(field.getOrDefault("colors",List.of()));if(rawColors.size()>128)throw new IllegalArgumentException("At most 128 color regions");
        for(var m:rawColors){var a=FieldPackage.pos(m.get("min_xyz_m"));var b=FieldPackage.pos(m.get("max_xyz_m"));var low=new Vector3f(Math.min(a.x,b.x),Math.min(a.y,b.y),Math.min(a.z,b.z));var high=new Vector3f(Math.max(a.x,b.x),Math.max(a.y,b.y),Math.max(a.z,b.z));var rgba=FieldPackage.vector(m.get("rgba"),4);int[] out=new int[4];for(int i=0;i<4;i++){if(rgba[i]<0||rgba[i]>255||rgba[i]!=(int)rgba[i])throw new IllegalArgumentException("Color rgba must contain integers 0..255");out[i]=(int)rgba[i];}colors.add(new ColorRegion(low,high,out));}
        world.space().addTickListener(this);
    }
    private boolean robotHasLink(String link){try{robot.bodyForLink(link);return true;}catch(RuntimeException e){return false;}}
    private Transform pose(Binding binding){var parent=binding.spec.link().isEmpty()?new Transform(world.robotPointWorld(Vector3f.ZERO),world.getChassisRotation()):robot.linkFrameWorld(binding.spec.link());return binding.spec.pose().clone().combineWithParent(parent);}
    private PhysicsRayTestResult ray(Vector3f from,Vector3f to){return world.space().rayTest(from,to).stream().filter(hit->!ownBodies.contains(hit.getCollisionObject().nativeId())).min(Comparator.comparingDouble(PhysicsRayTestResult::getHitFraction)).orElse(null);}
    private Object read(Binding b) {
        var pose=pose(b);var at=pose.getTranslation();var direction=pose.getRotation().mult(Vector3f.UNIT_X);var hit=ray(at,at.add(direction.mult((float)b.spec.maxRange())));
        if(b.device instanceof SimDistanceSensor){double distance=hit==null?Double.POSITIVE_INFINITY:hit.getHitFraction()*b.spec.maxRange();return distance<b.spec.minRange()?Double.POSITIVE_INFINITY:Double.isFinite(distance)?Math.max(b.spec.minRange(),Math.min(b.spec.maxRange(),distance+b.random.nextGaussian()*b.spec.noiseM())):distance;}
        if(b.device instanceof SimColorSensor){if(hit==null||hit.getHitFraction()*b.spec.maxRange()<b.spec.minRange())return new int[]{0,0,0,0};return color(hit.getCollisionObject(),at.add(direction.mult((float)(hit.getHitFraction()*b.spec.maxRange()))));}
        if(b.device instanceof SimTouchSensor)return touching(b,at,direction);
        return detections(b,pose);
    }
    private int[] color(PhysicsCollisionObject body,Vector3f point) {
        for(var region:colors)if(point.x>=region.min.x&&point.x<=region.max.x&&point.y>=region.min.y&&point.y<=region.max.y&&point.z>=region.min.z&&point.z<=region.max.z)return region.rgba.clone();
        if(body instanceof RigidBodyControl control&&control.getSpatial()!=null){var value=color(control.getSpatial());if(value!=null)return new int[]{Math.round(value.r*255),Math.round(value.g*255),Math.round(value.b*255),Math.round(value.a*255)};}
        return new int[]{0,0,0,0};
    }
    private ColorRGBA color(Spatial spatial) {
        if(spatial instanceof Geometry geometry&&geometry.getMaterial()!=null)for(String key:List.of("Diffuse","Color")){var p=geometry.getMaterial().getParam(key);if(p!=null&&p.getValue() instanceof ColorRGBA value)return value;}
        if(spatial instanceof Node node)for(var child:node.getChildren()){var value=color(child);if(value!=null)return value;}return null;
    }
    private boolean touching(Binding binding,Vector3f at,Vector3f direction) {
        long owner=binding.spec.link().isEmpty()?world.chassisBody().nativeId():robot.bodyForLink(binding.spec.link()).nativeId();int scanned=0;
        for(long m:world.space().listManifoldIds()){if(scanned++>=4096)break;long a=PersistentManifolds.getBodyAId(m),b=PersistentManifolds.getBodyBId(m);if(a!=owner&&b!=owner)continue;
            for(long p:PersistentManifolds.listPointIds(m)){var point=new Vector3f();if(a==owner)ManifoldPoints.getPositionWorldOnA(p,point);else ManifoldPoints.getPositionWorldOnB(p,point);
                var normal=new Vector3f();ManifoldPoints.getNormalWorldOnB(p,normal);if(b==owner)normal.negateLocal();
                if(normal.dot(direction)<=-binding.spec.pressCos()&&ManifoldPoints.getDistance1(p)<=.001&&point.distance(at)<=binding.spec.radiusM()&&ManifoldPoints.getAppliedImpulse(p)/world.space().getAccuracy()>=binding.spec.thresholdN())return true;}}
        return false;
    }
    private List<AprilTagDetection> detections(Binding binding,Transform camera) {
        var out=new ArrayList<AprilTagDetection>();var at=camera.getTranslation();var inverse=camera.getRotation().inverse();
        for(var tag:tags) {
            Vector3f relative=inverse.mult(tag.pose.getTranslation().subtract(at));double distance=relative.length();
            if(distance<binding.spec.minRange()||distance>binding.spec.maxRange()||relative.x<=0||tag.pose.getRotation().mult(Vector3f.UNIT_X).dot(at.subtract(tag.pose.getTranslation()))<=0)continue;
            boolean visible=true;
            for(int v=0;v<5;v++){var corner=tag.pose.getTranslation().clone();if(v<4)corner.addLocal(tag.pose.getRotation().mult(new Vector3f(0,(float)(tag.size/2*(v<2?1:-1)),(float)(tag.size/2*(v%2==0?1:-1)))));
                var p=inverse.mult(corner.subtract(at));if(p.x<=0||Math.abs(Math.atan2(p.z,p.x))>binding.spec.horizontalFov()/2||Math.abs(Math.atan2(p.y,p.x))>binding.spec.verticalFov()/2){visible=false;break;}
                var obstruction=ray(at,corner);if(obstruction!=null&&obstruction.getHitFraction()*corner.distance(at)<corner.distance(at)-.002){visible=false;break;}}
            if(!visible)continue;
            var normal=inverse.mult(tag.pose.getRotation().mult(Vector3f.UNIT_X));var up=inverse.mult(tag.pose.getRotation().mult(Vector3f.UNIT_Y));
            double x=relative.z+binding.random.nextGaussian()*binding.spec.noiseM(),y=relative.x+binding.random.nextGaussian()*binding.spec.noiseM(),z=relative.y+binding.random.nextGaussian()*binding.spec.noiseM();
            var detection=new AprilTagDetection();detection.id=tag.id;detection.metadata=new AprilTagMetadata(tag.id,tag.name,tag.size,DistanceUnit.METER);detection.frameAcquisitionNanoTime=Math.round(clock*1e9);
            detection.ftcPose=new AprilTagPoseFtc(x,y,z,Math.atan2(normal.z,-normal.x),Math.atan2(-normal.y,Math.hypot(normal.x,normal.z)),Math.atan2(up.z,up.y),Math.hypot(x,y),Math.atan2(-x,y),Math.atan2(z,Math.hypot(x,y)));out.add(detection);
        }
        return List.copyOf(out);
    }
    private void publish(Binding b,Object value){if(b.device instanceof SimDistanceSensor d)d.setDistanceMeters((Double)value);else if(b.device instanceof SimColorSensor c){var rgba=(int[])value;c.setColor(rgba[0],rgba[1],rgba[2],rgba[3]);}else if(b.device instanceof SimTouchSensor t)t.setPressed((Boolean)value);else ((WebcamName)b.device).setSimulatedDetections(castDetections(value));}
    @SuppressWarnings("unchecked") private static List<AprilTagDetection> castDetections(Object value){return (List<AprilTagDetection>)value;}
    public void prePhysicsTick(PhysicsSpace space,float dt) { }
    public void physicsTick(PhysicsSpace space,float dt) {if(closed)return;clock+=dt;for(var binding:bindings){if(clock+1e-9>=binding.next){binding.queue.add(new Pending(clock+binding.spec.latencyS(),clock,read(binding)));binding.next+=1/binding.spec.updateHz();if(binding.next<clock)binding.next=clock+1/binding.spec.updateHz();}while(!binding.queue.isEmpty()&&binding.queue.peek().available<=clock+1e-9)publish(binding,binding.queue.remove().value);}}
    List<String> summaries(){return bindings.stream().map(b->b.spec.name()+" ("+b.spec.type()+") "+(b.device instanceof SimDistanceSensor d?String.format(Locale.ROOT,"%.3f m",d.getDistance(DistanceUnit.METER)):b.device instanceof SimTouchSensor t?t.isPressed()?"pressed":"released":b.device instanceof SimColorSensor c?"RGBA "+c.red()+","+c.green()+","+c.blue()+","+c.alpha():((WebcamName)b.device).simulatedDetections().size()+" geometric tags")).toList();}
    public void close(){if(closed)return;closed=true;world.space().removeTickListener(this);for(var b:bindings){b.queue.clear();if(b.device instanceof WebcamName camera)camera.clearSimulatedCamera();else if(b.device instanceof SimDistanceSensor distance)distance.setDistanceMeters(Double.POSITIVE_INFINITY);else if(b.device instanceof SimTouchSensor touch)touch.setPressed(false);else if(b.device instanceof SimColorSensor color)color.setColor(0,0,0,0);}}
}
