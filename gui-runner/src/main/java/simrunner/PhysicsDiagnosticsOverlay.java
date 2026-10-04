package simrunner;

import com.jme3.asset.AssetManager;
import com.jme3.font.*;
import com.jme3.material.*;
import com.jme3.math.*;
import com.jme3.scene.*;
import com.jme3.scene.debug.Arrow;
import com.jme3.scene.shape.Quad;
import java.nio.file.*;
import java.util.*;
import java.util.function.Supplier;

/** Bounded, pooled visualization. Hidden overlays never poll native physics. */
final class PhysicsDiagnosticsOverlay {
    static final double UPDATE_S=.1,FORCE_METERS_PER_N=.01,MAX_ARROW_M=.35;
    final Node root=new Node("physics-diagnostic-vectors"),panel=new Node("physics-diagnostic-panel");
    final BitmapText text;
    private final Geometry background;
    private final AssetManager assets;
    private final Supplier<PhysicsDiagnostics.Snapshot> capture;
    private final Map<String,Material> materials=new HashMap<>();
    private final List<Geometry> arrows=new ArrayList<>();
    private double elapsed;
    private int used;
    private boolean visible;
    private PhysicsDiagnostics.Snapshot snapshot;
    private String saved="";
    PhysicsDiagnosticsOverlay(Node world,Node gui,AssetManager assets,BitmapFont font,Supplier<PhysicsDiagnostics.Snapshot> capture) {
        this.assets=assets;this.capture=capture;world.attachChild(root);gui.attachChild(panel);
        background=new Geometry("diagnostics-background",new Quad(1,1));var material=new Material(assets,"Common/MatDefs/Misc/Unshaded.j3md");material.setColor("Color",new ColorRGBA(.025f,.035f,.05f,.95f));material.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);background.setMaterial(material);panel.attachChild(background);
        text=new BitmapText(font);text.setColor(ColorRGBA.White);text.setLocalTranslation(12,-12,2);panel.attachChild(text);setVisible(false);
    }
    boolean visible(){return visible;}
    void setVisible(boolean value){visible=value;root.setCullHint(value?Spatial.CullHint.Inherit:Spatial.CullHint.Always);panel.setCullHint(value?Spatial.CullHint.Inherit:Spatial.CullHint.Always);elapsed=UPDATE_S;}
    void update(float seconds,int width,int height) {
        if(!visible)return;elapsed+=Math.max(0,seconds);if(elapsed<UPDATE_S)return;elapsed=0;
        snapshot=capture.get();render(snapshot,width,height);
    }
    private Material material(String name,ColorRGBA color){return materials.computeIfAbsent(name,n->{var m=new Material(assets,"Common/MatDefs/Misc/Unshaded.j3md");m.setColor("Color",color);m.getAdditionalRenderState().setWireframe(true);m.getAdditionalRenderState().setLineWidth(2);m.getAdditionalRenderState().setDepthTest(false);m.getAdditionalRenderState().setDepthWrite(false);return m;});}
    private void arrow(String label,Vector3f origin,Vector3f direction,ColorRGBA color) {
        if(direction.lengthSquared()<1e-10)return;
        Geometry g;if(used>=arrows.size()){g=new Geometry("diagnostic-vector",new Arrow(direction));root.attachChild(g);arrows.add(g);}else g=arrows.get(used);
        ((Arrow)g.getMesh()).setArrowExtent(direction);g.setMaterial(material(label,color));g.setLocalTranslation(origin);g.setCullHint(Spatial.CullHint.Inherit);used++;
    }
    static Vector3f scaledForce(Vector3f force){return force.length()>MAX_ARROW_M/FORCE_METERS_PER_N?force.normalize().mult((float)MAX_ARROW_M):force.mult((float)FORCE_METERS_PER_N);}
    private void render(PhysicsDiagnostics.Snapshot s,int width,int height) {
        used=0;
        for(var c:s.contacts()){var normal=c.normal().vector();arrow("normal",c.position().vector(),normal.mult(.08f),ColorRGBA.Green);arrow("load",c.position().vector(),scaledForce(normal.mult((float)c.loadN())),ColorRGBA.Yellow);}
        Vector3f forward=s.forward().vector(),left=new Vector3f(forward.z,0,-forward.x);
        for(var w:s.wheels())if(w.forceN()!=null)arrow("tire",w.hub().vector(),scaledForce(forward.mult(w.forceN().floatValue()).add(left.mult(w.lateralN().floatValue()))),ColorRGBA.Red);
        arrow("drive",s.driveOrigin().vector(),scaledForce(s.driveForce().vector()),new ColorRGBA(.25f,.5f,1,1));
        for(var j:s.joints())arrow("axis",j.pivot().vector(),j.axis().vector().mult(.12f),ColorRGBA.Cyan);
        for(int i=used;i<arrows.size();i++)arrows.get(i).setCullHint(Spatial.CullHint.Always);
        float panelWidth=Math.min(460,width*.42f),panelHeight=Math.max(250,height-120);panel.setLocalTranslation(width-panelWidth-15,height-110,1);
        background.setLocalTranslation(0,-panelHeight,0);background.setLocalScale(panelWidth,panelHeight,1);
        text.setSize(Math.min(14,Math.max(10,height/60f)));text.setBox(new com.jme3.font.Rectangle(0,0,panelWidth-24,panelHeight-24));text.setText(summary(s)+(saved.isEmpty()?"":"\n"+saved));
    }
    private static String shortName(String name){return name.length()>18?name.substring(0,15)+"...":name;}
    private static String f(double x){return String.format(Locale.ROOT,"%.3g",x);}
    static String summary(PhysicsDiagnostics.Snapshot s) {
        var t=new StringBuilder("PHYSICS DIAGNOSTICS  |  D: hide  P: save\n");
        t.append("Last tick: ").append(f(s.timestepS()*1000)).append(" ms | speed ").append(f(s.speedMps())).append(" m/s\n");
        t.append("Yaw rate ").append(f(s.yawRateRadS())).append(" rad/s | drive yaw ").append(f(s.driveTorqueNm())).append(" N*m\n");
        for(var note:s.notes().stream().limit(3).toList())t.append(note).append('\n');
        if(s.notes().size()>3)t.append("More notes in saved snapshot (P).\n");
        t.append("Green: contact normal | Yellow: normal load\nRed: tire force | Blue: drive | Cyan: joint axis\nForce: 1 N = 1 cm; arrows capped at 35 cm\nNormals: 8 cm; axes: 12 cm (direction only)\n");
        t.append("Wheel support / load / grip limit (N)\n");
        for(var w:s.wheels().stream().limit(4).toList()){t.append(shortName(w.joint())).append(w.supported()?" ON ":" OFF ").append(f(w.loadN())).append(" / ").append(f(w.gripN())).append(" N\n");if(w.slipMps()!=null)t.append("  slip ").append(f(w.slipMps())).append(" m/s | force ").append(f(w.forceN())).append(" N").append(w.sliding()?" | sliding":"").append('\n');}
        if(s.wheels().isEmpty())t.append("No configured native drive wheels.\n");
        if(s.wheels().size()>4)t.append("More wheels in saved snapshot (P).\n");
        t.append("Motors: command, rad/s, modeled N*m, A\n");
        for(var m:s.motors().stream().limit(4).toList())t.append(shortName(m.name())).append(' ').append(f(m.command())).append(' ').append(f(m.speedRadS())).append(' ').append(f(m.torqueNm())).append(' ').append(f(m.currentA())).append('\n');
        if(s.motors().size()>4)t.append("More motors in saved snapshot (P).\n");
        t.append("Mechanisms: position, limits, effort\n");
        for(var j:s.joints().stream().limit(4).toList())t.append(shortName(j.name())).append(' ').append(f(j.position())).append(' ').append(j.unit()).append(j.lower()==null?" (continuous)":" ["+f(j.lower())+", "+f(j.upper())+"]").append(j.effortIsLimit()?"\n  effort limit ":"\n  effort ").append(f(j.effort())).append(j.unit().equals("m")?" N":" N*m").append(" | rate ").append(f(j.velocity())).append(' ').append(j.unit()).append("/s\n");
        if(s.joints().isEmpty())t.append("No articulated mechanisms.\n");if(s.joints().size()>4)t.append("More mechanisms in saved snapshot (P).\n");
        t.append("Contacts shown: ").append(s.contacts().size()).append(s.truncated()?" (sampling limited)":"").append("\nModeled values; physical accuracy requires measurement.");return t.toString();
    }
    Path save(Path folder)throws Exception {
        snapshot=capture.get();Path file=folder.resolve("snapshot-"+UUID.randomUUID()+".json");
        try{ProfileIO.save(file,PhysicsDiagnostics.data(snapshot));saved="Saved diagnostics snapshot (build/physics-diagnostics).";return file;}
        catch(Exception e){saved="Cannot save diagnostics: "+e.getMessage();throw e;}finally{elapsed=UPDATE_S;}
    }
    PhysicsDiagnostics.Snapshot snapshot(){return snapshot;}
    int vectorCount(){return used;}
    void dispose(){root.removeFromParent();panel.removeFromParent();arrows.clear();materials.clear();used=0;}
}
