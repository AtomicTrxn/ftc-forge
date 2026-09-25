package simrunner;

import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.shape.Box;

/**
 * Minimal physics smoke test (not part of any phase's Definition of Done, just a de-risking
 * step before building the full Phase 4 scene): drop a dynamic box onto a static floor and
 * confirm it settles near the floor instead of falling through it or never landing.
 */
public class PhysicsSmokeTestApp extends SimpleApplication {

    private BulletAppState bulletAppState;
    private RigidBodyControl fallingBoxControl;
    private int frameCount = 0;

    public static void main(String[] args) {
        PhysicsSmokeTestApp app = new PhysicsSmokeTestApp();
        app.setShowSettings(false);
        app.start();
    }

    @Override
    public void simpleInitApp() {
        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);

        Geometry floor = new Geometry("floor", new Box(5f, 0.1f, 5f));
        floor.setLocalTranslation(0, -0.1f, 0);
        floor.setMaterial(unshaded(ColorRGBA.Gray));
        RigidBodyControl floorControl = new RigidBodyControl(new BoxCollisionShape(new Vector3f(5f, 0.1f, 5f)), 0f); // mass 0 = static
        floor.addControl(floorControl);
        rootNode.attachChild(floor);
        bulletAppState.getPhysicsSpace().add(floorControl);

        Geometry fallingBox = new Geometry("fallingBox", new Box(0.2f, 0.2f, 0.2f));
        fallingBox.setLocalTranslation(0, 5f, 0);
        fallingBox.setMaterial(unshaded(ColorRGBA.Red));
        fallingBoxControl = new RigidBodyControl(new BoxCollisionShape(new Vector3f(0.2f, 0.2f, 0.2f)), 1f);
        fallingBox.addControl(fallingBoxControl);
        rootNode.attachChild(fallingBox);
        bulletAppState.getPhysicsSpace().add(fallingBoxControl);

        System.out.println("[SMOKE TEST] Physics space created: " + bulletAppState.getPhysicsSpace());
    }

    private Material unshaded(ColorRGBA color) {
        Material m = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        m.setColor("Color", color);
        return m;
    }

    @Override
    public void simpleUpdate(float tpf) {
        frameCount++;
        Vector3f pos = fallingBoxControl.getPhysicsLocation();
        if (frameCount % 15 == 0) {
            System.out.println("[SMOKE TEST] frame=" + frameCount + " boxY=" + pos.y);
        }
        if (frameCount > 120) {
            boolean settled = Math.abs(pos.y - 0.2f) < 0.05f; // box half-height above floor top (y=0)
            System.out.println("[SMOKE TEST] Final boxY=" + pos.y + " settled=" + settled);
            stop();
        }
    }
}
