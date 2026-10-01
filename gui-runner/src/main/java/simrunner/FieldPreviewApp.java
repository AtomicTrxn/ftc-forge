package simrunner;

import com.jme3.app.SimpleApplication;
import com.jme3.math.*;
import com.jme3.scene.*;
import com.jme3.material.Material;
import java.nio.file.*;
import java.util.*;
import simcore.RobotUrdf;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Visual-only source/practice audit; neither CAD model is resized to fit the view. */
public final class FieldPreviewApp extends SimpleApplication {
    private FieldPackage field;private Path robotProject,screenshotPath;private boolean sourceLayout,fullDetail;private int frames;
    private com.jme3.app.state.ScreenshotAppState screenshot;
    public static void main(String[] args)throws Exception {
        if(args.length<1)throw new IllegalArgumentException("Usage: FieldPreviewApp <field.json> [robotProject] [--source-layout] [--full-detail] [screenshot.png]");
        var app=new FieldPreviewApp();app.field=new FieldPackage(Path.of(args[0]));
        for(int i=1;i<args.length;i++)switch(args[i]){case "--source-layout"->app.sourceLayout=true;case "--full-detail"->app.fullDetail=true;default->{if(args[i].endsWith(".png"))app.screenshotPath=Path.of(args[i]).toAbsolutePath();else app.robotProject=Path.of(args[i]);}}
        var settings=new com.jme3.system.AppSettings(true);settings.setResolution(1280,800);settings.setTitle("FTC Forge — field and robot scale preview");app.setSettings(settings);app.setShowSettings(false);app.setPauseOnLostFocus(false);app.start();
    }
    @Override public void simpleInitApp(){try {
        rootNode.addLight(new com.jme3.light.AmbientLight(new ColorRGBA(.6f,.6f,.6f,1)));rootNode.addLight(new com.jme3.light.DirectionalLight(new Vector3f(-1,-2,-1).normalizeLocal(),ColorRGBA.White));
        Map<String,Mesh> cache=new HashMap<>();
        for(var inst:field.instances) {
            String category=FieldPackage.str(inst,"category"),id=FieldPackage.str(inst,"mesh");
            if(category.equals("reference")||category.equals("detail")&&!fullDetail)continue;
            var mesh=cache.get(id);
            if(mesh==null){Path path=field.resolve(FieldPackage.str(field.meshes.get(id),"file"));if(fullDetail)path=field.resolve(field.directory.relativize(field.sourceUrdf.getParent().getParent().resolve("meshes").resolve(id)).toString());
                mesh=StlMeshLoader.load(path,new double[]{1,1,1},fullDetail?1_000_000:100_000);cache.put(id,mesh);}
            Geometry geom=new Geometry(FieldPackage.str(inst,"id"),mesh);geom.setLocalTransform(FieldPackage.transform(sourceLayout&&inst.containsKey("source_pose")?FieldPackage.map(inst.get("source_pose")):inst));
            if(!sourceLayout && inst.containsKey("mesh_center_m"))geom.setLocalTranslation(geom.getLocalTranslation().subtract(geom.getLocalRotation().mult(FieldPackage.pos(inst.get("mesh_center_m")))));
            var color=FieldPackage.vector(inst.get("rgba"),4);var material=new Material(assetManager,"Common/MatDefs/Light/Lighting.j3md");material.setBoolean("UseMaterialColors",true);var rgba=new ColorRGBA((float)color[0],(float)color[1],(float)color[2],1);material.setColor("Diffuse",rgba);material.setColor("Ambient",rgba);material.getAdditionalRenderState().setFaceCullMode(com.jme3.material.RenderState.FaceCullMode.Off);geom.setMaterial(material);rootNode.attachChild(geom);
        }
        Node robot;
        if(robotProject!=null){var cfg=SimConfig.load(robotProject);if(cfg.urdf==null)throw new IllegalArgumentException("Preview project requires robot URDF");Path path=robotProject.resolve(cfg.urdf);var cad=new ImportedRobotScene(RobotUrdf.parse(path),path,new HardwareMap(),assetManager,8);robot=cad.root;robot.setLocalTranslation(cfg.robotStart==null?new Vector3f(-1.2f,(float)cfg.startHeightM,1.2f):cfg.robotStart);robot.setLocalRotation(new Quaternion().fromAngleAxis(cfg.robotYawRad,Vector3f.UNIT_Y));}
        else {robot=new Node("generic robot");var g=new Geometry("18-inch chassis",new com.jme3.scene.shape.Box(.2286f,.1f,.2286f));var m=new Material(assetManager,"Common/MatDefs/Misc/Unshaded.j3md");m.setColor("Color",ColorRGBA.Blue);g.setMaterial(m);robot.attachChild(g);robot.setLocalTranslation(-1.2f,.1f,1.2f);}
        rootNode.attachChild(robot);flyCam.setEnabled(false);cam.setFrustumPerspective(45,(float)cam.getWidth()/cam.getHeight(),.01f,50);
        Node target=new Node("field-center");target.setLocalTranslation(0,.3f,0);rootNode.attachChild(target);var orbit=new com.jme3.input.ChaseCamera(cam,target,inputManager);orbit.setDefaultDistance(6.5f);orbit.setMinDistance(.4f);orbit.setMaxDistance(15);orbit.setDefaultHorizontalRotation(.75f);orbit.setDefaultVerticalRotation(.65f);orbit.setDragToRotate(true);
        var text=new com.jme3.font.BitmapText(guiFont);text.setText(String.format(Locale.ROOT,"%s | %s layout | visual-only\nInterior %.5f × %.5f m | both models: meters, scale 1\nDrag to orbit | scroll to zoom",field.name,sourceLayout?"original CAD":"practice",field.halfExtents.x*2,field.halfExtents.z*2));text.setLocalTranslation(15,cam.getHeight()-15,0);guiNode.attachChild(text);setDisplayStatView(false);
        if(screenshotPath!=null){Files.createDirectories(screenshotPath.getParent());String name=screenshotPath.getFileName().toString();screenshot=new com.jme3.app.state.ScreenshotAppState(screenshotPath.getParent()+"/",name.substring(0,name.length()-4));screenshot.setIsNumbered(false);stateManager.attach(screenshot);}
    }catch(Exception e){throw new IllegalArgumentException("Field preview failed: "+e.getMessage(),e);}}
    @Override public void simpleUpdate(float dt){if(screenshot!=null){if(++frames==30)screenshot.takeScreenshot();if(frames==40)stop();}}
}
