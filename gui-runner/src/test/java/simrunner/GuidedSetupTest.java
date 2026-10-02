package simrunner;

import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;
import java.nio.file.*;
import java.util.*;
import java.util.zip.*;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.system.NativeLibraryLoader;
import simcore.MiniJson;

class GuidedSetupTest {
    @TempDir Path tmp;
    static final String ROBOT="<robot name=\"Different robot\"><link name=\"base\"><visual><geometry><box size=\".3 .2 .1\"/></geometry></visual></link></robot>";
    static final String FIELD="<robot name=\"Different field\"><link name=\"floor\"><visual><geometry><box size=\"2 2 .02\"/></geometry></visual></link><link name=\"ball\"><visual><geometry><sphere radius=\".04\"/></geometry></visual></link><joint name=\"mount\" type=\"fixed\"><parent link=\"floor\"/><child link=\"ball\"/><origin xyz=\".5 .5 .08\"/></joint></robot>";
    @BeforeAll static void nativeLibrary(){NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    GuidedSetupController controller()throws Exception{return new GuidedSetupController(tmp.resolve("library"),null);}
    Path source(String xml)throws Exception{Path zip=tmp.resolve(UUID.randomUUID()+".zip");try(var z=new ZipOutputStream(Files.newOutputStream(zip))){z.putNextEntry(new ZipEntry("cad/model.urdf"));z.write(xml.getBytes(java.nio.charset.StandardCharsets.UTF_8));z.closeEntry();}return zip;}
    void load(GuidedSetupController c,String kind)throws Exception{c.load(kind,"cad",source(kind.equals("robot")?ROBOT:FIELD),null);}
    void ready(GuidedSetupController c,String kind)throws Exception{try(var p=new ModelValidation.Prepared(c.session.modelPath(kind),new DesktopAssetManager(true))){p.writeProof();}Path out=Path.of(c.models.run("save",c.session.modelPath(kind).toString(),c.models.library.toString(),"--reviewed").toString());c.session.select(kind,out);}
    @Test void workflowHasConditionalStepsAndNeverTrustsCompletionFlags()throws Exception {
        var c=controller();c.session.start("robot",null);load(c,"robot");assertEquals("scale",c.session.firstIncomplete().page());
        var scale=new GuidedSetupSession.Step("robot","scale");c.session.acknowledge(scale);assertTrue(c.session.complete(scale));assertEquals("parts",c.session.firstIncomplete().page());
        c.session.data.put("finished",true);assertFalse(c.session.complete(new GuidedSetupSession.Step("robot","review")));assertThrows(IllegalArgumentException.class,()->c.review("robot",false));
        var p=c.session.profile("robot");FieldPackage.map(p.get("parameters")).put("friction",.9);c.update("robot",p);c.compile("robot");assertTrue(c.session.complete(scale));
        c.units("robot","mm");assertFalse(c.session.complete(scale));assertTrue(c.session.steps().stream().noneMatch(s->s.page().equals("scene")));
    }
    @Test void savedCadSkipsModelChecksAndEditKeepsOriginalRevision()throws Exception {
        var c=controller();c.session.start("simulation",null);load(c,"robot");ready(c,"robot");Path saved=c.session.modelPath("robot");String original=Files.readString(saved);
        c.session.genericField();assertEquals("scene",c.session.firstIncomplete().page());c.load("robot","cad",source(ROBOT),null);assertEquals(saved,c.session.modelPath("robot"));assertEquals("scene",c.session.firstIncomplete().page());
        c.edit("robot");assertFalse(c.session.ready("robot"));assertEquals(original,Files.readString(saved));assertEquals("scale",c.session.firstIncomplete().page());
    }
    @Test void resumeRecoversDirtyDraftAndInvalidatesChangedSettings()throws Exception {
        var c=controller();c.session.start("robot",null);load(c,"robot");var scale=new GuidedSetupSession.Step("robot","scale");c.session.acknowledge(scale);Path session=c.session.file;
        var p=c.session.profile("robot");FieldPackage.map(p.get("parameters")).put("origin_xyz_m",List.of(.2,0.,0.));c.update("robot",p);
        var resumed=controller();resumed.resume(session);assertEquals("scale",resumed.session.firstIncomplete().page());assertEquals(List.of(.2,0.,0.),FieldPackage.map(resumed.session.profile("robot").get("parameters")).get("origin_xyz_m"));
        Files.delete(resumed.session.modelPath("robot"));assertEquals("source",resumed.session.firstIncomplete().page());
    }
    @Test void scenePreflightChecksContactsWithoutProjectAndLayoutChangesInvalidateProof()throws Exception {
        var c=controller();c.session.start("simulation",null);load(c,"robot");ready(c,"robot");c.session.genericField();var scene=c.newScene();scene.put("robot_start_xyz_m",List.of(0.,0.,.08));Path file=c.saveScene(scene,false);
        try(var built=new ScenePreflight.Prepared(file,new DesktopAssetManager(true))){assertEquals(6,built.space.countRigidBodies());}
        c.session.scene(file,true);assertTrue(c.session.sceneReady());c.saveScene(scene,false);assertTrue(c.session.sceneReady(),"Unchanged preview save retains its native proof");
        scene.put("robot_start_xyz_m",List.of(0.,0.,0.));c.saveScene(scene,false);assertFalse(c.session.sceneReady());assertThrows(IllegalArgumentException.class,()->new ScenePreflight.Prepared(file,new DesktopAssetManager(true)));
        try(var preview=new ScenePreflight.Prepared(file,new DesktopAssetManager(true),false)){assertEquals(6,preview.space.countRigidBodies());}
    }
    @Test void genericPiecesCanBeDuplicatedDisabledAndResetAtSavedPoses()throws Exception {
        var c=controller();c.session.start("simulation",null);load(c,"robot");ready(c,"robot");c.session.genericField();var scene=c.newScene();scene.put("robot_start_xyz_m",List.of(-1.,-1.,.08));scene.put("mode","game-pieces");var layout=c.sourcePieces("biobuzz");String first=layout.keySet().iterator().next();var copy=new LinkedHashMap<>(FieldPackage.map(layout.get(first)));copy.put("xyz_m",List.of(1.,1.,.08));layout.put("copy",copy);FieldPackage.map(layout.get(first)).put("enabled",false);scene.put("pieces",layout);Path file=c.saveScene(scene,false);
        try(var built=new ScenePreflight.Prepared(file,new DesktopAssetManager(true))){assertEquals(6,built.world.gamePieces().size());var piece=built.world.gamePieces().stream().filter(p->p.id().equals("copy")).findFirst().orElseThrow();var start=piece.body().getPhysicsLocation();piece.body().setPhysicsLocation(new com.jme3.math.Vector3f());built.world.resetPieces();assertEquals(start,piece.body().getPhysicsLocation());}
        scene.put("robot_start_xyz_m",List.of(1.,1.,.08));c.saveScene(scene,false);assertThrows(IllegalArgumentException.class,()->new ScenePreflight.Prepared(file,new DesktopAssetManager(true)));
        scene.put("robot_start_xyz_m",List.of(-1.,-1.,.08));FieldPackage.map(layout.get("copy")).put("xyz_m",List.of(1.82,1.,1.));c.saveScene(scene,false);
        var error=assertThrows(IllegalArgumentException.class,()->new ScenePreflight.Prepared(file,new DesktopAssetManager(true)));assertTrue(error.getMessage().contains("outside the field"),"Check the entire native shape, including a piece centered inside but extending beyond the boundary above the walls.");
    }
    @Test void importedFieldModesAndMissingTemplatesAreValidated()throws Exception {
        var c=controller();c.session.start("simulation",null);load(c,"robot");ready(c,"robot");load(c,"field");var p=c.session.profile("field");FieldPackage.map(FieldPackage.map(FieldPackage.map(p.get("entities")).get("ball")).get("settings")).put("role","piece");c.update("field",p);c.compile("field");ready(c,"field");var scene=c.newScene();scene.put("robot_start_xyz_m",List.of(-.4,-.4,.08));Path file=c.saveScene(scene,false);
        try(var built=new ScenePreflight.Prepared(file,new DesktopAssetManager(true))){assertEquals(0,built.world.gamePieces().size());}
        scene.put("mode","game-pieces");scene.put("pieces",c.sourcePieces("biobuzz"));c.saveScene(scene,false);try(var built=new ScenePreflight.Prepared(file,new DesktopAssetManager(true))){assertEquals(1,built.world.gamePieces().size());}
        scene.put("pieces",Map.of("gone",Map.of("source_id","missing","enabled",true,"xyz_m",List.of(0.,0.,.1),"rpy_rad",List.of(0.,0.,0.))));c.saveScene(scene,false);assertThrows(IllegalArgumentException.class,()->new ScenePreflight.Prepared(file,new DesktopAssetManager(true)));
    }
    @Test void projectCheckDoesNotWriteConfigOrStartTeamCodeAndApplyPreservesScene()throws Exception {
        var c=controller();c.session.start("simulation",null);load(c,"robot");ready(c,"robot");c.session.genericField();var scene=c.newScene();scene.put("robot_start_xyz_m",List.of(0.,0.,.08));Path file=c.saveScene(scene,false);c.session.scene(file,true);
        Path project=tmp.resolve("team");Files.createDirectories(project.resolve("TeamCode/src/main/java"));Path robotConfig=Path.of("sample-teamcode/robot_config.xml");if(!Files.exists(robotConfig))robotConfig=Path.of("gui-runner/sample-teamcode/robot_config.xml");Files.copy(robotConfig,project.resolve("robot_config.xml"));ProfileIO.save(project.resolve("preset.json"),Map.of("name","test","motors",Map.of()));ProfileIO.save(project.resolve("sim.config"),Map.of("robotConfig","robot_config.xml","presetMotors","preset.json"));String original=Files.readString(project.resolve("sim.config"));
        Files.writeString(project.resolve("TeamCode/src/main/java/Example.java"),"import com.qualcomm.robotcore.eventloop.opmode.*; @TeleOp(name=\"Example\") public class Example extends OpMode { static { System.setProperty(\"ftc.guided.code.started\",\"yes\"); } public void init(){} public void loop(){} }");
        c.session.start("simulation",project);assertEquals(List.of("Example"),c.checkProject("profile"));assertEquals(original,Files.readString(project.resolve("sim.config")));assertNull(System.getProperty("ftc.guided.code.started"));c.applyProject("profile");assertEquals(file.toRealPath(),Path.of(MiniJson.parseObject(Files.readString(project.resolve("sim.config"))).get("scene_profile").toString()));assertNotNull(SimConfig.load(project).robotProfile);
        c.session.start("robot",project);c.edit("robot");ready(c,"robot");c.applyProject("profile");assertTrue(MiniJson.parseObject(Files.readString(project.resolve("sim.config"))).containsKey("scene_profile"));assertEquals(c.session.modelPath("robot").toRealPath(),SimConfig.load(project).robotProfile.path);assertNull(System.getProperty("ftc.guided.code.started"));
    }
    @Test void invalidRuntimeDefaultsCannotPassGuidedPreparation()throws Exception {
        var c=controller();c.session.start("robot",null);load(c,"robot");var p=c.session.profile("robot");FieldPackage.map(p.get("runtime")).put("drive",Map.of("type","differential","left_motor","left","right_motor","right","track_width_m",-.3,"wheel_radius_m",.04,"left_shaft_sign",1,"right_shaft_sign",1));c.update("robot",p);assertThrows(IllegalArgumentException.class,()->c.compile("robot"));assertFalse(c.session.ready("robot"));
    }
    @Test void generatedFloorSupportsAnEmptyAssemblyFrameAfterPieceSeparation()throws Exception {
        var c=controller();c.session.start("field",null);c.load("field","cad",source(FIELD.replace("<link name=\"floor\"><visual><geometry><box size=\"2 2 .02\"/></geometry></visual></link>","<link name=\"floor\"/>")),null);var p=c.session.profile("field");FieldPackage.map(FieldPackage.map(FieldPackage.map(p.get("entities")).get("ball")).get("settings")).put("role","piece");c.update("field",p);c.compile("field");try(var built=new ModelValidation.Prepared(c.session.modelPath("field"),new DesktopAssetManager(true))){assertEquals(2,built.space.countRigidBodies());}
    }
    @Test void parameterLabelsAndSessionVersionProvideActionableContext()throws Exception {
        assertTrue(GuidedParameterPanel.label("entities/wheel/settings/mass_kg").contains("kg"));assertTrue(GuidedParameterPanel.help("collision_strategy").contains("openings"));var c=controller();c.session.data.put("schema_version",999);c.session.save();assertThrows(IllegalArgumentException.class,()->new GuidedSetupSession(c.session.file,true));
    }
    @Test void backgroundWorkRestoresSpinnerEditorsAndKeepsUnavailableControlsDisabled()throws Exception {
        javax.swing.SwingUtilities.invokeAndWait(()->{
            var panel=new javax.swing.JPanel();var spinner=new javax.swing.JSpinner(new javax.swing.SpinnerNumberModel(0.,-100.,100.,.01));var unavailable=new javax.swing.JButton("Unavailable");unavailable.setEnabled(false);panel.add(spinner);panel.add(unavailable);
            var editor=((javax.swing.JSpinner.DefaultEditor)spinner.getEditor()).getTextField();assertTrue(editor.isEnabled());var states=new IdentityHashMap<java.awt.Component,Boolean>();
            GuidedSetupWizard.setControls(panel,false,states);assertFalse(editor.isEnabled());GuidedSetupWizard.setControls(panel,true,states);assertTrue(spinner.isEnabled());assertTrue(editor.isEnabled(),"Users must still be able to type a placement after a scene check.");assertFalse(unavailable.isEnabled());assertTrue(states.isEmpty());
        });
    }
}
