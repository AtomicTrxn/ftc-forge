package simrunner;

import com.jme3.asset.AssetManager;
import com.jme3.math.*;
import com.jme3.scene.*;
import com.jme3.material.Material;

/** World-sized grid and URDF coordinate directions; never scales CAD. */
final class PreviewGuides {
    static Node build(AssetManager assets,float radius) {
        var node=new Node("metric-guides");float step=radius>1?.5f:.1f;int count=Math.min(20,Math.max(5,(int)Math.ceil(radius/step)));float extent=count*step;
        for(int i=-count;i<=count;i++){line(node,assets,new Vector3f(i*step,0,-extent),new Vector3f(i*step,0,extent),new ColorRGBA(.3f,.33f,.38f,1));line(node,assets,new Vector3f(-extent,0,i*step),new Vector3f(extent,0,i*step),new ColorRGBA(.3f,.33f,.38f,1));}
        float axis=Math.max(step*2,radius*.6f);line(node,assets,Vector3f.ZERO,new Vector3f(axis,0,0),ColorRGBA.Red);line(node,assets,Vector3f.ZERO,new Vector3f(0,0,-axis),ColorRGBA.Green);line(node,assets,Vector3f.ZERO,new Vector3f(0,axis,0),ColorRGBA.Blue);return node;
    }
    private static void line(Node parent,AssetManager assets,Vector3f a,Vector3f b,ColorRGBA color){var g=new Geometry("metric-axis",new com.jme3.scene.shape.Line(a,b));var m=new Material(assets,"Common/MatDefs/Misc/Unshaded.j3md");m.setColor("Color",color);g.setMaterial(m);parent.attachChild(g);}
}
