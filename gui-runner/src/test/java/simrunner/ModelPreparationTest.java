package simrunner;

import org.junit.jupiter.api.*;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.math.*;
import java.nio.file.*;
import java.util.*;
import java.util.zip.*;
import simcore.MiniJson;

class ModelPreparationTest {
    @TempDir Path tmp;
    static final String CAD="""
        <robot name="Unrelated fixture"><link name="floor"><visual><geometry><box size="2 2 .02"/></geometry></visual></link>
        <link name="gate"><visual><geometry><box size=".03 .4 .3"/></geometry></visual></link>
        <joint name="pivot" type="revolute"><parent link="floor"/><child link="gate"/><origin xyz=".7 0 .2"/><axis xyz="0 0 1"/><limit lower="-.5" upper=".5" effort="5" velocity="2"/></joint>
        <link name="ball"><visual><geometry><sphere radius=".04"/></geometry></visual></link>
        <joint name="ball_mount" type="fixed"><parent link="floor"/><child link="ball"/><origin xyz="0 0 .08"/></joint></robot>
        """;
    @BeforeAll static void nativeLibrary(){NativeLibraryLoader.loadNativeLibrary("bulletjme",true);}
    Path prepare(String kind)throws Exception {
        Path zip=tmp.resolve("source.zip");try(var z=new ZipOutputStream(Files.newOutputStream(zip))){z.putNextEntry(new ZipEntry("cad/urdf/model.urdf"));z.write(CAD.getBytes(java.nio.charset.StandardCharsets.UTF_8));z.closeEntry();}
        var controller=new ModelEditorController(tmp.resolve("library"));return Path.of(controller.run("import",zip.toString(),controller.library.toString(),"--kind",kind).toString());
    }
    Path reviewed(Path draft)throws Exception {
        try(var nativeModel=new ModelValidation.Prepared(draft,new DesktopAssetManager(true))){nativeModel.writeProof();}
        var c=new ModelEditorController(tmp.resolve("library"));return Path.of(c.run("save",draft.toString(),c.library.toString(),"--reviewed").toString());
    }
    @Test void sharedPreviewAndRuntimeUseMaterialsMarginsInertiaAndPinnedReview()throws Exception {
        Path draft=prepare("robot");var c=new ModelEditorController(tmp.resolve("library"));var data=MiniJson.parseObject(Files.readString(draft));var params=FieldPackage.map(data.get("parameters"));params.put("friction",.87);params.put("collision_margin_m",.001);ProfileIO.save(draft,data);c.run("compile",draft.toString());
        assertThrows(IllegalArgumentException.class,()->new ModelProfile(draft,true));
        try(var model=new ModelValidation.Prepared(draft,new DesktopAssetManager(true))){assertEquals(2,model.space.countRigidBodies());assertEquals(2,model.space.countJoints());for(var body:model.space.getRigidBodyList())assertEquals(.87,body.getFriction(),1e-6);model.writeProof();}
        Path saved=Path.of(c.run("save",draft.toString(),c.library.toString(),"--reviewed").toString());assertEquals("robot",new ModelProfile(saved,true).kind);
        Path editable=Path.of(c.run("edit",saved.toString(),c.library.toString()).toString());var changed=MiniJson.parseObject(Files.readString(editable));FieldPackage.map(changed.get("parameters")).put("friction",.3);ProfileIO.save(editable,changed);c.run("compile",editable.toString());assertThrows(IllegalArgumentException.class,()->new ModelProfile(editable,true));assertDoesNotThrow(()->new ModelProfile(saved,true));
        Files.writeString(new ModelProfile(saved,true).artifact("robot"),"tampered");assertThrows(IllegalArgumentException.class,()->new ModelProfile(saved,true));
    }
    @Test void arbitraryFieldHasStaticSupportPassiveJointRollingPieceAndReset()throws Exception {
        Path draft=prepare("field");var data=MiniJson.parseObject(Files.readString(draft));var entity=FieldPackage.map(FieldPackage.map(data.get("entities")).get("ball"));FieldPackage.map(entity.get("settings")).put("role","piece");ProfileIO.save(draft,data);var c=new ModelEditorController(tmp.resolve("library"));c.run("compile",draft.toString());Path saved=reviewed(draft);
        var space=new com.jme3.bullet.PhysicsSpace(com.jme3.bullet.PhysicsSpace.BroadphaseType.DBVT);try{var root=new com.jme3.scene.Node();var assets=new DesktopAssetManager(true);var world=new PhysicsWorld(assets,root,space);var field=new ModelFieldScene(new ModelProfile(saved,true),true,world,root,assets);assertEquals(3,space.countRigidBodies());assertEquals(1,space.countJoints());assertEquals(1,world.gamePieces().size());assertEquals(1,field.halfExtents.x,1e-6);var piece=world.gamePieces().get(0);assertEquals(new Vector3f(1,1,1),piece.body().getAngularFactor(null));var start=piece.body().getPhysicsLocation();piece.body().setPhysicsLocation(new Vector3f(.5f,.4f,.3f));field.reset();assertEquals(start,piece.body().getPhysicsLocation());for(int i=0;i<120;i++)space.update(1f/120,0);assertTrue(piece.body().getPhysicsLocation().y>.039);assertTrue(piece.body().getPhysicsLocation().y<.06);}finally{space.destroy();}
    }
    @Test void passiveSliderUsesLinearSpringUnits()throws Exception {
        Path draft=prepare("field");var data=MiniJson.parseObject(Files.readString(draft));var settings=FieldPackage.map(FieldPackage.map(FieldPackage.map(data.get("entities")).get("gate")).get("settings"));
        var joint=FieldPackage.map(settings.get("joint"));joint.put("type","prismatic");joint.put("axis",List.of(1.,0.,0.));settings.put("joint_spring_n_per_m",2.);settings.put("joint_damping_ns_per_m",.5);settings.put("joint_rest_m",.1);
        ProfileIO.save(draft,data);new ModelEditorController(tmp.resolve("library")).run("compile",draft.toString());
        try(var model=new ModelValidation.Prepared(draft,new DesktopAssetManager(true))){var moving=model.space.getRigidBodyList().stream().filter(b->b.isDynamic()).findFirst().orElseThrow();float before=moving.getPhysicsLocation().x;for(int i=0;i<480;i++)model.space.update(1f/480,0);assertTrue(moving.getPhysicsLocation().x>before+.02,"Linear spring should move slider toward its saved meter rest position");}
    }
    @Test void sceneDuplicatesUseSourceTemplatesAndPreservePieceCenterOfMass()throws Exception {
        Path draft=prepare("field");var data=MiniJson.parseObject(Files.readString(draft));var entity=FieldPackage.map(FieldPackage.map(data.get("entities")).get("ball"));var settings=FieldPackage.map(entity.get("settings"));settings.put("role","piece");settings.put("com_xyz_m",List.of(.02,0.,0.));ProfileIO.save(draft,data);var c=new ModelEditorController(tmp.resolve("library"));c.run("compile",draft.toString());Path saved=reviewed(draft);
        var p=new ModelProfile(saved,true);String source=FieldPackage.str(FieldPackage.maps(FieldPackage.map(p.receipt.get("artifacts")).get("pieces")).get(0),"id");
        Map<String,Object> layout=Map.of(source,Map.of("enabled",false),"copy-a",Map.of("source_id",source,"xyz_m",List.of(.4,0.,.08),"rpy_rad",List.of(0.,0.,0.)),"copy-b",Map.of("source_id",source,"xyz_m",List.of(-.4,0.,.08),"rpy_rad",List.of(0.,0.,0.)));
        var space=new com.jme3.bullet.PhysicsSpace(com.jme3.bullet.PhysicsSpace.BroadphaseType.DBVT);try{var root=new com.jme3.scene.Node();var assets=new DesktopAssetManager(true);var world=new PhysicsWorld(assets,root,space);var field=new ModelFieldScene(p,true,world,root,assets,layout);assertEquals(2,world.gamePieces().size());assertEquals(4,space.countRigidBodies());assertEquals(.42,world.gamePieces().stream().filter(x->x.id().equals("copy-a")).findFirst().orElseThrow().body().getPhysicsLocation().x,1e-6);field.reset();assertEquals(2,world.gamePieces().size());assertThrows(IllegalArgumentException.class,()->new ModelFieldScene(p,true,world,root,assets,Map.of("missing",Map.of("source_id","gone","enabled",true))));}finally{space.destroy();}
    }
    @Test void scenePinsIdentityAndDoesNotAlterModelSettings()throws Exception {
        Path saved=reviewed(prepare("robot"));var c=new ModelEditorController(tmp.resolve("library"));Path scene=tmp.resolve("scene.json");c.run("scene",scene.toString(),"--robot",saved.toString());var original=Files.readString(saved);var config=new LinkedHashMap<String,Object>(Map.of("scene_profile",scene.toString(),"model_settings","profile"));ProfileIO.save(tmp.resolve("sim.config"),config);var loaded=SimConfig.load(tmp);assertNotNull(loaded.robotProfile);assertEquals(original,Files.readString(saved));var data=MiniJson.parseObject(Files.readString(scene));FieldPackage.map(data.get("robot_identity")).put("revision_id","missing");ProfileIO.save(scene,data);assertThrows(java.io.IOException.class,()->SimConfig.load(tmp));
    }
    @Test void selectingModelUpdatesActiveSceneAndKeepsPlacements()throws Exception {
        Path first=reviewed(prepare("robot"));var c=new ModelEditorController(tmp.resolve("library"));Path draft=Path.of(c.run("edit",first.toString(),c.library.toString()).toString());var data=MiniJson.parseObject(Files.readString(draft));FieldPackage.map(data.get("parameters")).put("friction",.93);ProfileIO.save(draft,data);c.run("compile",draft.toString());Path second=reviewed(draft);Path scene=tmp.resolve("scene.json");c.run("scene",scene.toString(),"--robot",first.toString());var layout=MiniJson.parseObject(Files.readString(scene));layout.put("robot_start_xyz_m",List.of(.2,.3,.1));ProfileIO.save(scene,layout);ProfileIO.save(tmp.resolve("sim.config"),Map.of("scene_profile",scene.toString()));ProjectModelSelection.apply(tmp,new ModelProfile(second,true),"profile");var loaded=SimConfig.load(tmp);assertEquals(second.toRealPath(),loaded.robotProfile.path);assertEquals(new Vector3f(.2f,.1f,-.3f),loaded.robotStart);
    }
    @Test void projectConflictsRequireAnExplicitResolutionChoice()throws Exception {
        Path draft=prepare("robot");var data=MiniJson.parseObject(Files.readString(draft));FieldPackage.map(data.get("runtime")).put("start_height_m",.2);ProfileIO.save(draft,data);new ModelEditorController(tmp.resolve("library")).run("compile",draft.toString());Path saved=reviewed(draft);var config=new LinkedHashMap<String,Object>(Map.of("robot_model_profile",saved.toString(),"start_height_m",.1));ProfileIO.save(tmp.resolve("sim.config"),config);assertThrows(java.io.IOException.class,()->SimConfig.load(tmp));config.put("model_settings","profile");ProfileIO.save(tmp.resolve("sim.config"),config);assertEquals(.2,SimConfig.load(tmp).startHeightM);config.put("model_settings","project");ProfileIO.save(tmp.resolve("sim.config"),config);assertEquals(.1,SimConfig.load(tmp).startHeightM);
    }
}
