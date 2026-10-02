package simrunner;

import com.jme3.app.SimpleApplication;
import com.jme3.math.*;
import com.jme3.bounding.BoundingBox;
import com.jme3.font.BitmapText;
import java.nio.file.*;

/** A scene can be inspected before project hardware or TeamCode is supplied. */
public final class GuidedScenePreview extends SimpleApplication {
    private Path file,png;private ScenePreflight.Prepared scene;private int frames;
    private com.jme3.app.state.ScreenshotAppState screenshot;
    public static void main(String[] args){var app=new GuidedScenePreview();app.file=Path.of(args[0]);if(args.length>1)app.png=Path.of(args[1]).toAbsolutePath();var settings=new com.jme3.system.AppSettings(true);settings.setTitle("FTC Forge — scene setup preview");settings.setResolution(1280,800);app.setSettings(settings);app.setShowSettings(false);app.setPauseOnLostFocus(false);app.start();}
    @Override public void simpleInitApp(){
        try {
            scene=new ScenePreflight.Prepared(file,assetManager,false);rootNode.attachChild(scene.root);rootNode.addLight(new com.jme3.light.AmbientLight(new ColorRGBA(.65f,.65f,.65f,1)));rootNode.addLight(new com.jme3.light.DirectionalLight(new Vector3f(-1,-2,-1).normalizeLocal(),ColorRGBA.White));rootNode.updateGeometricState();
            var overlay=new CollisionOverlay(scene.space,rootNode,assetManager);overlay.setVisible(true);overlay.update();float radius=2;Vector3f center=new Vector3f();if(scene.root.getWorldBound() instanceof BoundingBox b){radius=Math.max(.1f,Math.max(b.getXExtent(),Math.max(b.getYExtent(),b.getZExtent())));center=b.getCenter().clone();}
            rootNode.attachChild(PreviewGuides.build(assetManager,radius));flyCam.setEnabled(false);cam.setFrustumPerspective(45,(float)cam.getWidth()/cam.getHeight(),.001f,Math.max(30,radius*20));var orbit=new com.jme3.input.ChaseCamera(cam,scene.root,inputManager);orbit.setLookAtOffset(center);orbit.setDefaultDistance(radius*2.5f);orbit.setMinDistance(.2f);orbit.setMaxDistance(radius*10);orbit.setDragToRotate(true);
            var text=new BitmapText(guiFont);text.setLocalTranslation(15,cam.getHeight()-15,0);text.setText("Scene setup | native shapes at metric scale 1 | TeamCode not started\nDrag: orbit · scroll: zoom | red X forward, green Y left, blue Z up\nPreview does not approve placement. Return to the wizard and Check scene.");guiNode.attachChild(text);setDisplayStatView(false);setDisplayFps(false);
            if(png!=null){Files.createDirectories(png.getParent());String name=png.getFileName().toString();screenshot=new com.jme3.app.state.ScreenshotAppState(png.getParent()+"/",name.substring(0,name.length()-4));screenshot.setIsNumbered(false);stateManager.attach(screenshot);}
        }catch(Exception e){throw new IllegalArgumentException("Scene preview: "+e.getMessage(),e);}
    }
    @Override public void simpleUpdate(float dt){if(screenshot!=null){if(++frames==15)screenshot.takeScreenshot();if(frames==25)stop();}}
    @Override public void destroy(){if(scene!=null)scene.close();super.destroy();}
}
