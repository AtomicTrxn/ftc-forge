package simrunner;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.*;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bounding.BoundingBox;
import com.jme3.math.*;
import com.jme3.scene.Node;
import com.jme3.system.NativeLibraryLoader;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import simcore.*;
import java.nio.file.*;
import java.util.*;
import static org.junit.jupiter.api.Assertions.*;

class CollisionReviewTest {
    @TempDir Path temp;
    @BeforeAll static void nativePhysics(){NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    private String link(String name,boolean collision) {
        return "<link name='"+name+"'><inertial><mass value='1'/><inertia ixx='.01' iyy='.02' izz='.02' ixy='0' ixz='0' iyz='0'/></inertial><visual><geometry><box size='.1 .1 .1'/></geometry></visual>"+(collision?"<collision><geometry><box size='.1 .1 .1'/></geometry></collision>":"")+"</link>";
    }
    private String joint(String type,String parent,String child) {
        return "<joint name='joint_"+child+"' type='"+type+"'><parent link='"+parent+"'/><child link='"+child+"'/><origin xyz='.2 0 0'/>"+(type.equals("fixed")?"":"<axis xyz='1 0 0'/><limit lower='0' upper='.5'/>")+"</joint>";
    }
    private ImportedRobotScene scene(String xml)throws Exception {
        Path source=temp.resolve("robot.urdf");Files.writeString(source,"<robot name='test'>"+xml+"</robot>");
        return new ImportedRobotScene(RobotUrdf.parse(source),source,new HardwareMap(),new DesktopAssetManager(true),8);
    }
    @Test void missingMovingColliderFailsBeforeAddingAnyNativeBody()throws Exception {
        var scene=scene(link("base",true)+link("arm",false)+joint("prismatic","base","arm"));
        var space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            var audit=CollisionAudit.inspect(scene);assertFalse(audit.usable());assertTrue(audit.errors.get(0).contains("arm"));
            var error=assertThrows(IllegalArgumentException.class,()->new ArticulatedRobot(scene,new PhysicsWorld(null,new Node(),space),new Node(),Vector3f.ZERO));
            assertTrue(error.getMessage().contains("arm"));assertEquals(0,space.countRigidBodies());assertEquals(0,space.countJoints());
            Path report=temp.resolve("report.json");audit.write(report,temp.resolve("robot.urdf"));
            var json=MiniJson.parseObject(Files.readString(report));assertEquals(false,json.get("coverage_usable"));assertTrue(Files.readString(report).contains("missing"));
            String original=Files.readString(temp.resolve("robot.urdf"));
            assertThrows(IllegalArgumentException.class,()->audit.write(temp.resolve("robot.urdf"),temp.resolve("robot.urdf")));
            assertEquals(original,Files.readString(temp.resolve("robot.urdf")));
        }finally{space.destroy();}
    }
    @Test void addingColliderMakesPreviouslyUnprotectedSliderStopAtObstacle()throws Exception {
        var scene=scene(link("base",true)+link("arm",true)+joint("prismatic","base","arm"));
        var space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            space.setGravity(Vector3f.ZERO);var world=new PhysicsWorld(null,new Node(),space);
            var robot=new ArticulatedRobot(scene,world,new Node(),Vector3f.ZERO);world.chassisBody().setMass(0);
            var arm=robot.bodyForLink("arm");var obstacle=new PhysicsRigidBody(new BoxCollisionShape(new Vector3f(.05f,1,1)),0);obstacle.setPhysicsLocation(new Vector3f(.45f,0,0));space.add(obstacle);
            for(int i=0;i<240;i++){arm.applyCentralForce(new Vector3f(2,0,0));space.update(1f/120,0);}
            assertTrue(arm.getPhysicsLocation().x>.3f);assertTrue(arm.getPhysicsLocation().x<.36f,"Arm must stop at actual contact");
        }finally{space.destroy();}
    }
    @Test void fixedChildrenShareProxyButReportUnresolvedSurfaceCoverage()throws Exception {
        var scene=scene(link("base",false)+link("proxy",true)+link("detail",false)+joint("fixed","base","proxy")+joint("fixed","base","detail"));
        var audit=CollisionAudit.inspect(scene);assertTrue(audit.usable());assertEquals(1,audit.bodies.size());
        assertEquals("aggregate_proxy_review_required",audit.links.stream().filter(l->l.name().equals("detail")).findFirst().orElseThrow().treatment());
        assertFalse(scene.parts().get(0).shape() instanceof EmptyShape);
    }
    @Test void wheelOnlyDeclarationsCannotHideMissingChassisCollision()throws Exception {
        var scene=scene(link("base",false)+link("wheel",true)+joint("continuous","base","wheel")+"<transmission name='tx'><joint name='joint_wheel'/><actuator name='left_front_drive'/></transmission>");
        var audit=CollisionAudit.inspect(scene);assertFalse(audit.usable());assertEquals(0,audit.bodies.get(0).declarations());
        assertEquals("drive_wheel_ballast",audit.links.stream().filter(l->l.name().equals("wheel")).findFirst().orElseThrow().treatment());
    }
    @Test void explicitNoncontactMechanismIsAllowedAndReported()throws Exception {
        var scene=scene(link("base",true)+link("sensor",false)+joint("prismatic","base","sensor"));scene.collisionOmissions=Map.of("sensor","Internal sensor carriage intentionally does not contact the environment");
        var audit=CollisionAudit.inspect(scene);assertTrue(audit.usable());assertTrue(audit.bodies.stream().anyMatch(b->b.status().equals("intentional_noncontact")));
        var space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try{new ArticulatedRobot(scene,new PhysicsWorld(null,new Node(),space),new Node(),Vector3f.ZERO);assertEquals(2,space.countRigidBodies());}finally{space.destroy();}
    }
    @Test void invalidOrRedundantExemptionsFail()throws Exception {
        var scene=scene(link("base",true)+link("arm",false)+joint("prismatic","base","arm"));
        for(var omission:List.of(Map.of("base","reason"),Map.of("typo","reason"),Map.of("arm"," "))){scene.collisionOmissions=omission;assertFalse(CollisionAudit.inspect(scene).usable());}
        Path config=temp.resolve("sim.config");Files.writeString(config,"{\"urdf\":\"robot.urdf\",\"collision_omissions\":{\"arm\":4}}");assertThrows(IllegalArgumentException.class,()->SimConfig.load(temp));
    }
    @Test void nativeOverlayTracksRotatedCompoundOffsetsAndDoesNotAddBodies() {
        var space=new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        try {
            var shape=new CompoundCollisionShape();var box=new BoxCollisionShape(new Vector3f(.1f,.2f,.3f));box.setMargin(.001f);shape.addChildShape(box,new Vector3f(.4f,0,0));
            var body=new PhysicsRigidBody(shape,1);body.setPhysicsLocation(new Vector3f(1,2,3));body.setPhysicsRotation(new Quaternion().fromAngleAxis(.8f,Vector3f.UNIT_Y));space.add(body);
            Node root=new Node();var overlay=new CollisionOverlay(space,root,new DesktopAssetManager(true));overlay.setVisible(true);overlay.update();root.updateGeometricState();
            // A compound's broadphase AABB adds padding; that is not a contact surface.
            // Compare the real child's native surface at its composed world transform.
            var center=body.getPhysicsLocation().add(body.getPhysicsRotation().mult(new Vector3f(.4f,0,0)));
            var nativeBounds=box.boundingBox(center,body.getPhysicsRotation(),null);var shown=(BoundingBox)overlay.root.getWorldBound();
            assertEquals(0,nativeBounds.getCenter().distance(shown.getCenter()),.002);assertEquals(nativeBounds.getXExtent(),shown.getXExtent(),.003);assertEquals(nativeBounds.getZExtent(),shown.getZExtent(),.003);
            body.setPhysicsLocation(new Vector3f(-2,4,5));overlay.update();root.updateGeometricState();assertEquals(-2,overlay.root.getChild(0).getLocalTranslation().x,.00001);assertEquals(1,space.countRigidBodies());
            space.remove(body);overlay.update();assertEquals(0,overlay.shapeCount());
        }finally{space.destroy();}
    }
}
