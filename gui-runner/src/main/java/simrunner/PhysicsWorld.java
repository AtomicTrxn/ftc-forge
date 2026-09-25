package simrunner;

import com.jme3.asset.AssetManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Torus;
import vhacd4.Vhacd4;
import vhacd4.Vhacd4Hull;
import vhacd4.Vhacd4Parameters;

import java.util.List;

/**
 * Phase 4's rigid-body physics world (Libbulletjme via Minie, per R2). Field walls and game
 * pieces are real Bullet rigid bodies; the chassis is a dynamic rigid body driven by a
 * force/torque computed from the kinematics/motor model (per R2/R6's Mecanum constraint) --
 * NOT by wheel-ground contact, which no plain rigid-body engine can produce for a Mecanum
 * drivetrain's 45-degree roller behavior.
 */
public class PhysicsWorld {

    private static final boolean DEBUG_INTAKE = false; // flip to true to trace piece/intake-point distance each tick

    private static final float FIELD_HALF_SIZE_M = 1.8288f; // 12 ft / 2
    private static final float WALL_HEIGHT_M = 0.3f;        // illustrative, not sourced from a specific game's real field wall height
    private static final float WALL_THICKNESS_M = 0.02f;

    private final AssetManager assetManager;
    private final Node rootNode;
    private final BulletAppState bulletAppState;
    private final PhysicsSpace physicsSpace;

    private RigidBodyControl chassisControl;
    private RigidBodyControl gamePieceControl;
    private Node gamePieceNode;
    private Node chassisVisualNode;
    private boolean pieceHeld = false;

    public PhysicsWorld(AssetManager assetManager, Node rootNode, BulletAppState bulletAppState) {
        this.assetManager = assetManager;
        this.rootNode = rootNode;
        this.bulletAppState = bulletAppState;
        this.physicsSpace = bulletAppState.getPhysicsSpace();
    }

    public void buildFieldBoundary() {
        addStaticBox("floor", new Vector3f(FIELD_HALF_SIZE_M, 0.01f, FIELD_HALF_SIZE_M),
            new Vector3f(0, -0.01f, 0), ColorRGBA.DarkGray);

        addStaticBox("wall-north", new Vector3f(FIELD_HALF_SIZE_M, WALL_HEIGHT_M, WALL_THICKNESS_M),
            new Vector3f(0, WALL_HEIGHT_M, FIELD_HALF_SIZE_M), ColorRGBA.LightGray);
        addStaticBox("wall-south", new Vector3f(FIELD_HALF_SIZE_M, WALL_HEIGHT_M, WALL_THICKNESS_M),
            new Vector3f(0, WALL_HEIGHT_M, -FIELD_HALF_SIZE_M), ColorRGBA.LightGray);
        addStaticBox("wall-east", new Vector3f(WALL_THICKNESS_M, WALL_HEIGHT_M, FIELD_HALF_SIZE_M),
            new Vector3f(FIELD_HALF_SIZE_M, WALL_HEIGHT_M, 0), ColorRGBA.LightGray);
        addStaticBox("wall-west", new Vector3f(WALL_THICKNESS_M, WALL_HEIGHT_M, FIELD_HALF_SIZE_M),
            new Vector3f(-FIELD_HALF_SIZE_M, WALL_HEIGHT_M, 0), ColorRGBA.LightGray);
    }

    private void addStaticBox(String name, Vector3f halfExtents, Vector3f position, ColorRGBA color) {
        Geometry geom = new Geometry(name, new Box(halfExtents.x, halfExtents.y, halfExtents.z));
        geom.setMaterial(unshaded(color));
        geom.setLocalTranslation(position);
        RigidBodyControl control = new RigidBodyControl(new BoxCollisionShape(halfExtents), 0f); // mass 0 = static
        geom.addControl(control);
        rootNode.attachChild(geom);
        physicsSpace.add(control);
    }

    /**
     * Builds a torus-shaped game piece with collision geometry from real V-HACD convex
     * decomposition (per Phase 4's spec: "never collide against a raw imported mesh
     * directly"), not a hand-picked primitive shape. A torus is a deliberately non-convex
     * test case -- its own convex hull would fill in the donut hole, while V-HACD's multiple
     * convex hulls approximate the ring shape reasonably -- exercising the real pipeline
     * rather than a shape that wouldn't have needed it.
     */
    public void buildGamePiece(Vector3f position) {
        Torus torusMesh = new Torus(24, 12, 0.03f, 0.09f);
        gamePieceNode = new Node("game-piece");
        Geometry geom = new Geometry("game-piece-mesh", torusMesh);
        geom.setMaterial(unshaded(ColorRGBA.Orange));
        gamePieceNode.attachChild(geom);
        gamePieceNode.setLocalTranslation(position);
        rootNode.attachChild(gamePieceNode);

        CollisionShape collisionShape = buildVhacdShape(torusMesh);
        gamePieceControl = new RigidBodyControl(collisionShape, 0.1f); // ~100g, a light game element
        gamePieceNode.addControl(gamePieceControl);
        gamePieceControl.setPhysicsLocation(position);
        physicsSpace.add(gamePieceControl);
    }

    /** Real V-HACD decomposition (Vhacd4, bundled in Minie/Libbulletjme per R2) -- not a bounding-box or single-hull approximation. */
    private CollisionShape buildVhacdShape(Torus mesh) {
        java.nio.FloatBuffer posBuf = mesh.getFloatBuffer(com.jme3.scene.VertexBuffer.Type.Position);
        posBuf.rewind();
        float[] positions = new float[posBuf.remaining()];
        posBuf.get(positions);

        com.jme3.scene.mesh.IndexBuffer idxBuf = mesh.getIndexBuffer();
        int[] indices = new int[idxBuf.size()];
        for (int i = 0; i < indices.length; i++) indices[i] = idxBuf.get(i);

        // Default maxHulls is 64 -- far more than a simple torus needs, and a real cause of
        // instability found by actually colliding another body against the result (excessive
        // hull counts produced an explosive contact reaction with the chassis). 8 hulls is
        // still enough to approximate the ring shape's concavity while being far more robust.
        Vhacd4Parameters params = new Vhacd4Parameters();
        params.setMaxHulls(8);
        List<Vhacd4Hull> hulls = Vhacd4.compute(positions, indices, params);

        CompoundCollisionShape compound = new CompoundCollisionShape();
        for (Vhacd4Hull hull : hulls) {
            compound.addChildShape(new HullCollisionShape(hull));
        }
        System.out.println("[PHYSICS] V-HACD decomposed game piece into " + hulls.size() + " convex hull(s).");
        return compound;
    }

    public void buildChassis(Node visualNode, double massKg, Vector3f startPosition) {
        this.chassisVisualNode = visualNode;
        CollisionShape shape = new BoxCollisionShape(new Vector3f(0.2286f, 0.1f, 0.2286f));
        chassisControl = new RigidBodyControl(shape, (float) massKg);
        chassisControl.setPhysicsLocation(startPosition);
        // No linear/angular damping: chassis velocity is now directly commanded each tick
        // (see driveChassis), so damping would just fight that command rather than model
        // anything real.
        visualNode.addControl(chassisControl);
        physicsSpace.add(chassisControl);
    }

    /**
     * Drives the chassis by directly setting its linear/angular velocity from the kinematics
     * model's desired chassis-frame velocity, per R2/R6's Mecanum constraint (the kinematics
     * model, not wheel-ground contact, is authoritative for drivetrain motion). Bullet's
     * contact solver still resolves collisions normally on top of this each step (a directly
     * driven dynamic body is a standard, well-established technique for "kinematic-ish but
     * collidable" vehicles in physics-engine-backed sims).
     *
     * DEVIATION (documented): an earlier version of this method applied a bounded
     * force/torque instead, matching the plan's original "force and torque" framing more
     * literally. That approach was tried first and found to fight the chassis's own damping
     * (undershooting the kinematics target speed substantially) and, before the bound was
     * added, produced a genuine numerical explosion that cascaded into breaking the game
     * piece's V-HACD collision in the same shared physics space -- caught by actually running
     * the full scene and watching both the chassis and an unrelated body diverge to absurd
     * coordinates, not assumed safe from the isolated smoke tests alone. Direct velocity
     * control is simpler, has no gain to mis-tune, and cannot itself inject the same kind of
     * force-magnitude instability.
     */
    public void driveChassis(physics.MecanumKinematics.ChassisVelocity desiredRobotFrameVelocity, float dtSeconds) {
        if (dtSeconds <= 0) return;
        com.jme3.math.Quaternion rot = chassisControl.getPhysicsRotation();
        Vector3f desiredWorld = rot.mult(new Vector3f(
            (float) desiredRobotFrameVelocity.vx, 0, (float) -desiredRobotFrameVelocity.vy));
        chassisControl.setLinearVelocity(desiredWorld);
        chassisControl.setAngularVelocity(new Vector3f(0, (float) desiredRobotFrameVelocity.omega, 0));
    }

    public Vector3f getChassisPosition() { return chassisControl.getPhysicsLocation(); }
    public com.jme3.math.Quaternion getChassisRotation() { return chassisControl.getPhysicsRotation(); }

    /**
     * Proximity-trigger intake (per Phase 4's spec: an acceptable simplified model, not full
     * contact dynamics). When active and the game piece is within captureRadius of the
     * intake point, the piece is removed from free physics and parented to the chassis
     * visually; deactivating releases it back into free physics at its held position.
     */
    public void updateIntake(boolean intakeActive, Vector3f intakePointWorld, float captureRadiusM) {
        if (gamePieceControl == null) return;

        if (intakeActive && !pieceHeld) {
            float distance = gamePieceControl.getPhysicsLocation().distance(intakePointWorld);
            if (DEBUG_INTAKE) System.out.println("[PHYSICS DEBUG] piece=" + gamePieceControl.getPhysicsLocation()
                + " intakePoint=" + intakePointWorld + " distance=" + distance);
            if (distance < captureRadiusM) {
                physicsSpace.remove(gamePieceControl);
                pieceHeld = true;
                System.out.println("[PHYSICS] Game piece captured at distance " + distance + "m (< " + captureRadiusM + "m).");
            }
        } else if (!intakeActive && pieceHeld) {
            physicsSpace.add(gamePieceControl);
            gamePieceControl.setLinearVelocity(Vector3f.ZERO);
            pieceHeld = false;
            System.out.println("[PHYSICS] Game piece released.");
        }

        if (pieceHeld) {
            gamePieceNode.setLocalTranslation(intakePointWorld);
        }
    }

    public boolean isPieceHeld() { return pieceHeld; }
    public Vector3f getGamePiecePosition() { return pieceHeld ? gamePieceNode.getLocalTranslation() : gamePieceControl.getPhysicsLocation(); }

    private Material unshaded(ColorRGBA color) {
        Material m = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        m.setColor("Color", color);
        return m;
    }
}
