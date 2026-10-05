package simrunner;

import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import java.nio.file.*;
import java.util.*;
import static org.junit.jupiter.api.Assertions.*;
class FieldBehaviorTest {
    @TempDir Path tmp;
    @BeforeAll static void nativeLibrary(){com.jme3.system.NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    RobotMotionDemo demo()throws Exception{return new RobotMotionDemo(SyntheticRobots.importRobot(tmp,true,false,false,SyntheticRobots.Dimensions.standard()).motionSetup(),new DesktopAssetManager(true));}
    Map<String,Object> rule(String id,String mode,String containment){return Map.of("id",id,"alliance","red","mode",mode,"containment",containment,"types",List.of("pollen"),"min_xyz_m",List.of(.5,-.2,0),"max_xyz_m",List.of(1.,.2,.5),"points",2,"until_s",.2);}
    Map<String,Object> config(Object... rules){return Map.of("schema_version",1,"clock",Map.of("auto_s",.1,"transition_s",.05,"teleop_s",.15),"rules",List.of(rules));}
    RigidBodyControl piece(RobotMotionDemo d){var body=new RigidBodyControl(new SphereCollisionShape(.04f),.02f);body.setPhysicsLocation(new Vector3f(.75f,.2f,0));d.space.add(body);d.space.setGravity(Vector3f.ZERO);d.world.registerPiece("p","pollen",new Node(),body);return body;}
    void tick(RobotMotionDemo d,int n){for(int i=0;i<n;i++)d.space.update(1f/120,0);}
    @Test void occupancyCanBeRemovedAndEntryAwardsOnlyOnceWithPauseResetAndExport()throws Exception {
        try(var d=demo()){var body=piece(d);try(var game=new FieldBehavior(d.world,config(rule("live","occupancy","center"),rule("once","entry_once","center")),true)) {
            tick(d,2);assertEquals(0,game.score("red"));game.toggle();tick(d,2);assertEquals(4,game.score("red"));body.setPhysicsLocation(new Vector3f(1.1f,.2f,0));tick(d,1);assertEquals(2,game.score("red"));
            body.setPhysicsLocation(new Vector3f(.75f,.2f,0));tick(d,1);assertEquals(4,game.score("red"));game.toggle();var before=game.report().get("seconds");tick(d,5);assertEquals(before,game.report().get("seconds"));
            var file=game.export(tmp.resolve("reports"));assertEquals(4L,((Map<?,?>)simcore.MiniJson.parseObject(Files.readString(file)).get("scores")).get("red"));game.reset();assertEquals(0,game.score("red"));assertEquals("AUTO",game.phase());assertEquals(List.of(),game.report().get("events"));game.toggle();tick(d,1);assertEquals(4,game.score("red"));
        }}
    }
    @Test void partialFullBoundsSnapshotWindowAndFieldOnlyAreExplicit()throws Exception {
        try(var d=demo()){var body=piece(d);body.setPhysicsLocation(new Vector3f(1.02f,.2f,0));var partial=rule("partial","occupancy","partial");var full=rule("full","occupancy","full");var snap=rule("end","snapshot","center");
            try(var game=new FieldBehavior(d.world,config(partial,full,snap),true)){game.toggle();tick(d,1);assertEquals(2,game.score("red"));body.setPhysicsLocation(new Vector3f(.75f,.2f,0));tick(d,13);assertEquals("TRANSITION",game.phase());assertEquals(4,game.score("red"));tick(d,12);assertEquals(6,game.score("red"));body.setPhysicsLocation(new Vector3f(1.2f,.2f,0));tick(d,20);assertEquals(6,game.score("red"));assertEquals("FINISHED",game.phase());}
            try(var game=new FieldBehavior(d.world,config(partial),false)){body.setPhysicsLocation(new Vector3f(.75f,.2f,0));game.toggle();tick(d,4);assertEquals(0,game.score("red"));assertEquals(false,game.report().get("pieces_enabled"));}
        }
    }
    @Test void badRulesAndMissingOrDisplacedReferencePosesFailOrReportUnverified() {
        assertThrows(IllegalArgumentException.class,()->FieldBehavior.validate(config(rule("duplicate","occupancy","center"),rule("duplicate","occupancy","center"))));
        var bad=new LinkedHashMap<>(rule("bad","entry_once","center"));bad.put("points",.5);assertThrows(IllegalArgumentException.class,()->FieldBehavior.validate(config(bad)));
        var instances=new ArrayList<Map<String,Object>>();for(int i=0;i<4;i++)instances.add(Map.of("id","flower"+i,"mesh","am_5857__Flower_Layer_C.stl","xyz_m",List.of(i,0,0)));
        var field=Map.<String,Object>of("instances",instances,"meshes",Map.of(),"hives",List.of(Map.of(),Map.of()));var ref=Map.<String,Object>of("source_url","https://example.org/layout","revision","test fixture","tolerance_m",.005,"instances",List.of(Map.of("id","flower0","xyz_m",List.of(0,0,0)),Map.of("id","flower1","xyz_m",List.of(2,0,0)),Map.of("id","missing","xyz_m",List.of(0,0,0))));
        var audit=FieldLayoutAudit.audit(field,ref);assertEquals(false,audit.get("competition_layout_verified"));var checks=(List<Map<String,Object>>)audit.get("checks");assertEquals(true,checks.stream().filter(c->c.get("check").equals("pose flower0")).findFirst().orElseThrow().get("pass"));assertEquals(false,checks.stream().filter(c->c.get("check").equals("pose flower1")).findFirst().orElseThrow().get("pass"));assertTrue(checks.stream().anyMatch(c->"missing".equals(c.get("error_m"))));
    }
}
