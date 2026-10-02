package simrunner;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.font.*;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.*;
import com.jme3.math.*;
import com.jme3.scene.Spatial;
import com.jme3.system.NativeLibraryLoader;
import java.nio.file.*;
import java.util.*;

/** Import-time scripted physics demonstration. Report is separate from model review evidence. */
public final class RobotMotionDemoApp extends SimpleApplication {
    private Path file,project,report;
    private String selection="";
    private RobotMotionDemo demo;
    private CollisionOverlay overlay;
    private com.jme3.input.ChaseCamera orbit;
    private BitmapText status;
    private double accumulator;
    private boolean cad=true,reported;
    private String error="";
    public static void main(String[] args)throws Exception {
        if(args.length<3)throw new IllegalArgumentException("Usage: RobotMotionDemoApp <profile.json> <project|-> <report.json> [--headless] [--only <movement group>]");
        boolean headless=false;String selection="";
        for(int i=3;i<args.length;i++){if(args[i].equals("--headless"))headless=true;else if(args[i].equals("--only")&&i+1<args.length&&selection.isEmpty())selection=args[++i];else throw new IllegalArgumentException("Unknown or incomplete option: "+args[i]);}
        Path file=Path.of(args[0]).toAbsolutePath(),project=args[1].equals("-")?null:Path.of(args[1]).toAbsolutePath(),report=Path.of(args[2]).toAbsolutePath();
        if(!report.getFileName().toString().endsWith(".json")||report.normalize().startsWith(file.getParent().normalize())||Files.isSymbolicLink(report))throw new IllegalArgumentException("Demo report must be a separate session .json file outside the model directory.");
        if(headless) {
            NativeLibraryLoader.loadNativeLibrary("bulletjme",true);
            boolean reportWritten=false;
            try(var demo=new RobotMotionDemo(RobotMotionDemo.setup(file,project),new DesktopAssetManager(true),selection)) {
                while(!demo.finished())demo.tick();ProfileIO.save(report,demo.report());reportWritten=true;System.out.println("[MOTION DEMO] "+ProfileIO.json(demo.report()));
                if(!demo.failure.isEmpty())throw new IllegalStateException(demo.failure);
            }catch(Exception e){if(!reportWritten)ProfileIO.save(report,Map.of("status","needs attention","error",e.getMessage()==null?e.toString():e.getMessage()));throw e;}
            return;
        }
        var app=new RobotMotionDemoApp();app.file=file;app.project=project;app.report=report;app.selection=selection;
        var settings=new com.jme3.system.AppSettings(true);settings.setTitle("FTC Forge — robot motion demo");settings.setResolution(1280,800);settings.setFrameRate(60);app.setSettings(settings);app.setShowSettings(false);app.setPauseOnLostFocus(false);app.start();
    }
    public void simpleInitApp() {
        flyCam.setEnabled(false);setDisplayStatView(false);viewPort.setBackgroundColor(new ColorRGBA(.12f,.14f,.18f,1));
        status=new BitmapText(guiFont);status.setSize(17);status.setBox(new com.jme3.font.Rectangle(0,0,cam.getWidth()-30,cam.getHeight()-30));status.setLocalTranslation(15,cam.getHeight()-15,0);guiNode.attachChild(status);
        inputManager.addMapping("PauseDemo",new KeyTrigger(KeyInput.KEY_SPACE));inputManager.addMapping("StopDemo",new KeyTrigger(KeyInput.KEY_S));inputManager.addMapping("ReplayDemo",new KeyTrigger(KeyInput.KEY_R));inputManager.addMapping("DemoCAD",new KeyTrigger(KeyInput.KEY_V));inputManager.addMapping("DemoShapes",new KeyTrigger(KeyInput.KEY_C));
        inputManager.addListener((ActionListener)(name,pressed,tpf)->{if(!pressed)return;switch(name){case "ReplayDemo"->load();case "PauseDemo"->{if(demo!=null)demo.pause();}case "StopDemo"->{if(demo!=null){demo.stop();write();}}case "DemoShapes"->{if(overlay!=null)overlay.setVisible(!overlay.visible);}case "DemoCAD"->{if(demo!=null){cad=!cad;for(var child:demo.root.getChildren())child.setCullHint(cad?Spatial.CullHint.Inherit:Spatial.CullHint.Always);}}}} ,"PauseDemo","StopDemo","ReplayDemo","DemoCAD","DemoShapes");
        load();
    }
    private void load() {
        try {
            if(demo!=null){demo.root.removeFromParent();demo.close();demo=null;}if(overlay!=null){overlay.root.removeFromParent();overlay=null;}
            var fresh=new RobotMotionDemo(RobotMotionDemo.setup(file,project),assetManager,selection);demo=fresh;
            // Construction updates detached nodes. Local lights refresh their cached light lists on first load.
            demo.root.addLight(new com.jme3.light.AmbientLight(new ColorRGBA(.6f,.6f,.6f,1)));
            demo.root.addLight(new com.jme3.light.DirectionalLight(new Vector3f(-1,-2,-1).normalizeLocal(),ColorRGBA.White));
            rootNode.attachChild(demo.root);demo.root.updateGeometricState();
            Spatial old=rootNode.getChild("metric-guides");if(old!=null)old.removeFromParent();var grid=PreviewGuides.build(assetManager,2);rootNode.attachChild(grid);
            cam.setFrustumPerspective(45,(float)cam.getWidth()/cam.getHeight(),.005f,100);
            if(orbit==null)orbit=new com.jme3.input.ChaseCamera(cam,demo.robot.chassisNode,inputManager);else orbit.setSpatial(demo.robot.chassisNode);
            float size=1;if(demo.root.getWorldBound() instanceof com.jme3.bounding.BoundingBox box)size=Math.max(.25f,Math.max(box.getXExtent(),Math.max(box.getYExtent(),box.getZExtent())));
            orbit.setDefaultDistance(size*4);orbit.setMinDistance(.2f);orbit.setMaxDistance(20);orbit.setDefaultHorizontalRotation(.8f);orbit.setDefaultVerticalRotation(.5f);orbit.setDragToRotate(true);orbit.setTrailingEnabled(false);
            overlay=new CollisionOverlay(demo.space,rootNode,assetManager);overlay.setVisible(false);overlay.update();
            cad=true;accumulator=0;reported=false;error="";write();
        }catch(Exception e){if(demo!=null){demo.root.removeFromParent();demo.close();demo=null;}error=e.getMessage()==null?e.toString():e.getMessage();try{ProfileIO.save(report,Map.of("status","needs attention","error",error));}catch(Exception ignored){}}
    }
    private void write(){if(demo!=null)try{ProfileIO.save(report,demo.report());}catch(Exception e){error="Cannot save demo observations: "+e.getMessage();}}
    public void simpleUpdate(float dt) {
        if(demo!=null) {
            // Bounded work per rendered frame: slow the demo down instead of skipping native steps.
            accumulator=Math.min(.1,accumulator+dt);int ticks=0;
            while(accumulator>=RobotMotionDemo.DT&&ticks++<48){demo.tick();accumulator-=RobotMotionDemo.DT;}
            if(overlay!=null)overlay.update();
            if(demo.finished()&&!reported){write();reported=true;}
            var last=demo.observations.stream().skip(Math.max(0,demo.observations.size()-5)).map(row->row.get("action")+": "+row.get("outcome")).toList();
            status.setText(demo.setup.profile().name+" — "+demo.status()+"\n"+demo.setup.hardwareLabel()+"\n"
                +"Space: pause/resume | S: stop | R: replay | Esc: close | C: collision shapes | V: CAD\n"
                +"Neutral floor, actual native physics. Short bounded inputs; compare with your expectations.\n"
                +"Progress: "+demo.completedMovements()+" / "+demo.actions.size()+" movements\n"
                +String.join("\n",last)+(demo.setup.plan().notes.isEmpty()?"":"\n"+String.join("\n",demo.setup.plan().notes.stream().limit(3).toList()))
                +"\n"+(error.isEmpty()?"Observations saved for Guided setup → View demo results. This does not approve the model.":error));
        } else status.setText("Motion demo needs setup\n"+error+"\nReturn to Parts / Physics assumptions to correct settings. R: retry | Esc: close");
    }
    public void destroy(){if(demo!=null){if(!demo.finished())demo.stop();write();demo.close();}super.destroy();}
}
