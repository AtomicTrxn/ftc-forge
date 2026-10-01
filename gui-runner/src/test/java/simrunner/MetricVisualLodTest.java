package simrunner;

import com.jme3.scene.*;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class MetricVisualLodTest {
    @Test void boundsVisualMovementAndDoesNotMutateSourcePhysicsMesh() {
        Mesh source=new Mesh();float[] positions={.00013f,.00012f,0,.10023f,0,0,0,.20021f,0};
        source.setBuffer(VertexBuffer.Type.Position,3,positions);source.setBuffer(VertexBuffer.Type.Index,3,new int[]{0,1,2});
        var result=MetricVisualLod.cluster(source,.0005f);
        assertNotSame(source,result);assertEquals(1,result.getTriangleCount());
        for(int v=0;v<3;v++){double distance2=0;for(int i=0;i<3;i++){int k=v*3+i;float original=source.getFloatBuffer(VertexBuffer.Type.Position).get(k);assertEquals(positions[k],original);double delta=original-result.getFloatBuffer(VertexBuffer.Type.Position).get(k);distance2+=delta*delta;}assertTrue(Math.sqrt(distance2)<=.0004331);}
    }
    @Test void rejectsExcessiveSimplificationAndRetainsIsolatedThinDetail() {
        Mesh mesh=new Mesh();mesh.setBuffer(VertexBuffer.Type.Position,3,new float[]{0,0,0,.0001f,0,0,0,.0001f,0});mesh.setBuffer(VertexBuffer.Type.Index,3,new int[]{0,1,2});
        assertSame(mesh,MetricVisualLod.cluster(mesh,.0005f));
        assertThrows(IllegalArgumentException.class,()->MetricVisualLod.cluster(mesh,.01f));
    }
}
