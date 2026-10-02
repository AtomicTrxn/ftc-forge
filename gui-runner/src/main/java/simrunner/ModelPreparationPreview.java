package simrunner;

import com.jme3.app.SimpleApplication;
import com.jme3.math.*;
import com.jme3.bounding.BoundingBox;
import com.jme3.font.BitmapText;
import com.jme3.input.controls.*;
import com.jme3.input.KeyInput;
import com.jme3.scene.*;
import java.nio.file.*;
import java.util.Map;

/** Live profile preview. Editor compilation replaces receipt atomically; view reloads that revision. */
public final class ModelPreparationPreview extends SimpleApplication {
    private String modelStatus="";private Path file,png,view;private String viewHash="";private Node guides;private String receiptHash="";private float refresh;private int frames;
    private ModelValidation.Prepared model;private CollisionOverlay overlay;private BitmapText status;
    private Node fallback;
    private com.jme3.input.ChaseCamera orbit;private boolean cad=true;
    private com.jme3.app.state.ScreenshotAppState screenshot;
    public static void main(String[] args){
        if(args.length<1||args.length>4)throw new IllegalArgumentException("Usage: ModelPreparationPreview <draft/profile.json> [screenshot.png]");
        var app=new ModelPreparationPreview();app.file=Path.of(args[0]).toAbsolutePath();for(int i=1;i<args.length;i++){if(args[i].equals("--view"))app.view=Path.of(args[++i]);else app.png=Path.of(args[i]).toAbsolutePath();}
        var settings=new com.jme3.system.AppSettings(true);settings.setTitle("FTC Forge — model preparation");settings.setResolution(1280,800);app.setSettings(settings);app.setShowSettings(false);app.setPauseOnLostFocus(false);app.start();
    }
    @Override public void simpleInitApp(){
        rootNode.addLight(new com.jme3.light.AmbientLight(new ColorRGBA(.4f,.4f,.4f,1)));rootNode.addLight(new com.jme3.light.DirectionalLight(new Vector3f(-1,-2,-1).normalizeLocal(),ColorRGBA.White));
        status=new BitmapText(guiFont);status.setLocalTranslation(15,cam.getHeight()-15,0);guiNode.attachChild(status);flyCam.setEnabled(false);setDisplayStatView(false);
        viewPort.setBackgroundColor(new ColorRGBA(.14f,.16f,.19f,1));
        inputManager.addMapping("CAD",new KeyTrigger(KeyInput.KEY_V));inputManager.addMapping("Shapes",new KeyTrigger(KeyInput.KEY_C));
        inputManager.addListener((ActionListener)(name,pressed,tpf)->{if(pressed&&model!=null){if(name.equals("CAD")){cad=!cad;for(var c:model.root.getChildren())if(c!=overlay.root)c.setCullHint(cad?Spatial.CullHint.Inherit:Spatial.CullHint.Always);}else overlay.setVisible(!overlay.visible);}},"CAD","Shapes");
        reload();
        if(png!=null)try{Files.createDirectories(png.getParent());String name=png.getFileName().toString();screenshot=new com.jme3.app.state.ScreenshotAppState(png.getParent()+"/",name.substring(0,name.length()-4));screenshot.setIsNumbered(false);stateManager.attach(screenshot);}catch(Exception e){throw new RuntimeException(e);}
    }
    private void reload(){
        try {
            String next=ModelProfile.hash(file.getParent().resolve("receipt.json"));if(next.equals(receiptHash))return;receiptHash=next;
            var fresh=new ModelValidation.Prepared(file,assetManager);
            if(fallback!=null){fallback.removeFromParent();fallback=null;}
            if(model!=null){model.root.removeFromParent();model.close();}model=fresh;cad=true;model.root.addLight(new com.jme3.light.AmbientLight(new ColorRGBA(.65f,.65f,.65f,1)));model.root.addLight(new com.jme3.light.DirectionalLight(new Vector3f(-1,-2,-1).normalizeLocal(),ColorRGBA.White));rootNode.attachChild(model.root);
            model.root.updateGeometricState();
            float radius=1;Vector3f center=new Vector3f();if(model.root.getWorldBound() instanceof BoundingBox box){radius=Math.max(.01f,Math.max(box.getXExtent(),Math.max(box.getYExtent(),box.getZExtent())));center=box.getCenter().clone();}
            if(guides!=null)guides.removeFromParent();guides=PreviewGuides.build(assetManager,radius);rootNode.attachChild(guides);viewHash="";
            cam.setFrustumPerspective(45,(float)cam.getWidth()/cam.getHeight(),Math.max(.001f,radius/100),Math.max(10,radius*100));
            if(orbit==null)orbit=new com.jme3.input.ChaseCamera(cam,model.root,inputManager);else orbit.setSpatial(model.root);orbit.setLookAtOffset(center);orbit.setDefaultDistance(radius*3.5f);orbit.setMinDistance(radius*.2f);orbit.setMaxDistance(radius*20);orbit.setDefaultHorizontalRotation(.75f);orbit.setDefaultVerticalRotation(.5f);orbit.setDragToRotate(true);
            overlay=new CollisionOverlay(model.space,model.root,assetManager);overlay.setVisible(true);overlay.update();model.writeProof();
            modelStatus=model.profile.name+" | "+model.profile.kind+" | "+model.space.countRigidBodies()+" native bodies\nMeters, scale 1 | red X forward, green Y left, blue Z up\nDrag: orbit | scroll: zoom | C: shapes | V: CAD\nUnpowered preview; inspect gaps, openings and thin surfaces before Save reviewed.";status.setText(modelStatus);
        }catch(Exception e){
            if(model!=null){model.root.removeFromParent();model.close();model=null;overlay=null;}
            if(fallback!=null)fallback.removeFromParent();
            try {
                var p=new ModelProfile(file,false);Path cad=p.kind.equals("robot")?p.artifact("robot"):p.legacyField()?p.resolve("source/model.urdf"):p.artifact("field");
                var scene=new ImportedRobotScene(simcore.RobotUrdf.parse(cad),cad,new com.qualcomm.robotcore.hardware.HardwareMap(),assetManager,8,java.util.Set.of(),.0005f);
                fallback=scene.root;rootNode.attachChild(fallback);fallback.updateGeometricState();
                if(fallback.getWorldBound() instanceof BoundingBox box){float radius=Math.max(.01f,Math.max(box.getXExtent(),Math.max(box.getYExtent(),box.getZExtent())));if(orbit==null)orbit=new com.jme3.input.ChaseCamera(cam,fallback,inputManager);else orbit.setSpatial(fallback);orbit.setLookAtOffset(box.getCenter());orbit.setDefaultDistance(radius*3.5f);orbit.setMinDistance(radius*.2f);orbit.setMaxDistance(radius*20);orbit.setDragToRotate(true);cam.setFrustumPerspective(45,(float)cam.getWidth()/cam.getHeight(),.001f,Math.max(10,radius*100));}
            }catch(Exception ignored){}
            status.setText("CAD only — native preparation needs attention: "+e.getMessage()+"\nCorrect editor settings; preview reloads automatically. This draft cannot be approved.");
            try{ProfileIO.save(file.getParent().resolve("validation.json"),Map.of("valid",false,"error",e.getMessage()==null?e.toString():e.getMessage()));}catch(Exception ignored){}
        }
    }

    private void focus(){
        if(view==null||model==null||!Files.isRegularFile(view))return;
        try{String hash=ModelProfile.hash(view);if(hash.equals(viewHash))return;viewHash=hash;var data=simcore.MiniJson.parseObject(Files.readString(view));String link=data.getOrDefault("link","").toString();String owner=FieldPackage.map(model.profile.receipt.get("body_grouping")).getOrDefault(link,link).toString();Spatial visual=model.root.getChild(link);while(visual!=null&&!visual.getName().startsWith("body-"))visual=visual.getParent();if(visual!=null)owner=visual.getName().substring(5);overlay.focus(owner);
            for(var body:model.space.getRigidBodyList())if(body instanceof com.jme3.bullet.control.RigidBodyControl c&&c.getSpatial()!=null&&c.getSpatial().getName().equals("body-"+owner)){orbit.setLookAtOffset(body.getPhysicsLocation());float radius=.2f;var bounds=com.jme3.bullet.util.DebugShapeFactory.getDebugShape(body.getCollisionShape());bounds.updateGeometricState();if(bounds.getWorldBound() instanceof BoundingBox b)radius=Math.max(.03f,Math.max(b.getXExtent(),Math.max(b.getYExtent(),b.getZExtent())));orbit.setDefaultDistance(radius*4);status.setText(modelStatus+"\nFocused: "+link+" (magenta body)");break;}
        }catch(Exception ignored){}
    }
    @Override public void simpleUpdate(float dt){refresh+=dt;if(refresh>1){refresh=0;reload();focus();}if(overlay!=null)overlay.update();if(screenshot!=null){if(++frames==15)screenshot.takeScreenshot();if(frames==25)stop();}}
    @Override public void destroy(){if(model!=null)model.close();super.destroy();}
}
