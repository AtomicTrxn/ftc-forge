package simrunner;

import com.jme3.scene.*;
import com.jme3.math.Vector3f;
import java.util.*;

/** Metric vertex clustering for visuals only; source physics meshes remain unchanged. */
final class MetricVisualLod {
    private record Vertex(long x,long y,long z) { }
    static Mesh cluster(Mesh source,float step) {
        if(!(step>0)||step>.001f)throw new IllegalArgumentException("Visual grid must be >0 and <=1 mm");
        var buffer=source.getFloatBuffer(VertexBuffer.Type.Position);var index=source.getIndexBuffer();
        List<Float> positions=new ArrayList<>();Set<Set<Vertex>> seen=new HashSet<>();
        for(int i=0;i<index.size();i+=3) {
            Vertex[] v=new Vertex[3];
            for(int j=0;j<3;j++){int k=index.get(i+j)*3;v[j]=new Vertex(Math.round(buffer.get(k)/(double)step),Math.round(buffer.get(k+1)/(double)step),Math.round(buffer.get(k+2)/(double)step));}
            if(v[0].equals(v[1])||v[1].equals(v[2])||v[2].equals(v[0]))continue;
            var face=Set.of(v);if(!seen.add(face))continue;
            Vector3f a=point(v[0],step),b=point(v[1],step),c=point(v[2],step);
            if(b.subtract(a).cross(c.subtract(a)).lengthSquared()<1e-20f)continue;
            for(var p:List.of(a,b,c)){positions.add(p.x);positions.add(p.y);positions.add(p.z);}
        }
        if(positions.isEmpty())return source; // Thin isolated details must not disappear entirely.
        float[] p=new float[positions.size()],normals=new float[p.length];int[] indices=new int[p.length/3];
        for(int i=0;i<p.length;i++)p[i]=positions.get(i);for(int i=0;i<indices.length;i++)indices[i]=i;
        for(int i=0;i<p.length;i+=9){Vector3f a=new Vector3f(p[i+3]-p[i],p[i+4]-p[i+1],p[i+5]-p[i+2]),b=new Vector3f(p[i+6]-p[i],p[i+7]-p[i+1],p[i+8]-p[i+2]);var n=a.cross(b).normalizeLocal();for(int j=0;j<3;j++){normals[i+j*3]=n.x;normals[i+j*3+1]=n.y;normals[i+j*3+2]=n.z;}}
        Mesh mesh=new Mesh();mesh.setBuffer(VertexBuffer.Type.Position,3,p);mesh.setBuffer(VertexBuffer.Type.Normal,3,normals);mesh.setBuffer(VertexBuffer.Type.Index,3,indices);mesh.setStatic();mesh.updateBound();return mesh;
    }
    private static Vector3f point(Vertex v,float step){return new Vector3f(v.x*step,v.y*step,v.z*step);}
}
