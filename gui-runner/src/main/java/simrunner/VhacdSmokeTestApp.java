package simrunner;

import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.collision.shapes.*;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Torus;
import vhacd4.Vhacd4;
import vhacd4.Vhacd4Hull;
import vhacd4.Vhacd4Parameters;

import java.nio.FloatBuffer;
import java.util.List;

/** Isolated debug: does a V-HACD-decomposed torus fall onto a floor correctly? */
public class VhacdSmokeTestApp extends SimpleApplication {
    private BulletAppState bulletAppState;
    private RigidBodyControl torusControl;
    private int frame = 0;

    public static void main(String[] args) {
        VhacdSmokeTestApp app = new VhacdSmokeTestApp();
        app.setShowSettings(false);
        app.start();
    }

    @Override
    public void simpleInitApp() {
        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);

        Geometry floor = new Geometry("floor", new Box(2f, 0.05f, 2f));
        floor.setLocalTranslation(0, -0.05f, 0);
        floor.setMaterial(mat(ColorRGBA.Gray));
        RigidBodyControl floorControl = new RigidBodyControl(new BoxCollisionShape(new Vector3f(2f, 0.05f, 2f)), 0f);
        floor.addControl(floorControl);
        rootNode.attachChild(floor);
        bulletAppState.getPhysicsSpace().add(floorControl);

        Torus torusMesh = new Torus(24, 12, 0.03f, 0.09f);
        Geometry torusGeom = new Geometry("torus", torusMesh);
        torusGeom.setMaterial(mat(ColorRGBA.Orange));
        torusGeom.setLocalTranslation(0, 1f, 0);
        rootNode.attachChild(torusGeom);

        FloatBuffer posBuf = torusMesh.getFloatBuffer(VertexBuffer.Type.Position);
        posBuf.rewind();
        float[] positions = new float[posBuf.remaining()];
        posBuf.get(positions);
        System.out.println("[VHACD DEBUG] position floats: " + positions.length + " first few: "
            + positions[0] + "," + positions[1] + "," + positions[2]);

        IndexBuffer idxBuf = torusMesh.getIndexBuffer();
        int[] indices = new int[idxBuf.size()];
        for (int i = 0; i < indices.length; i++) indices[i] = idxBuf.get(i);
        System.out.println("[VHACD DEBUG] index count: " + indices.length + " first few: "
            + indices[0] + "," + indices[1] + "," + indices[2]);

        Vhacd4Parameters params = new Vhacd4Parameters();
        List<Vhacd4Hull> hulls = Vhacd4.compute(positions, indices, params);
        System.out.println("[VHACD DEBUG] hull count: " + hulls.size());

        CompoundCollisionShape compound = new CompoundCollisionShape();
        for (Vhacd4Hull hull : hulls) {
            HullCollisionShape hullShape = new HullCollisionShape(hull);
            compound.addChildShape(hullShape);
        }

        torusControl = new RigidBodyControl(compound, 0.1f);
        torusGeom.addControl(torusControl);
        torusControl.setPhysicsLocation(new Vector3f(0, 1f, 0));
        bulletAppState.getPhysicsSpace().add(torusControl);

        System.out.println("[VHACD DEBUG] compound shape childCount=" + compound.listChildren().length);
    }

    @Override
    public void simpleUpdate(float tpf) {
        frame++;
        if (frame % 15 == 0) {
            System.out.println("[VHACD DEBUG] frame=" + frame + " torusY=" + torusControl.getPhysicsLocation().y);
        }
        if (frame > 120) {
            System.out.println("[VHACD DEBUG] FINAL torusY=" + torusControl.getPhysicsLocation().y);
            stop();
        }
    }

    private Material mat(ColorRGBA color) {
        Material m = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        m.setColor("Color", color);
        return m;
    }
}
