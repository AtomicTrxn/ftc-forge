package simrunner;

import com.jme3.asset.AssetManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
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
 * bounded traction impulses computed from the kinematics/motor model. External pushes,
 * gravity, and collision impulses remain under Bullet control.
 */
public class PhysicsWorld {

    private static final boolean DEBUG_INTAKE = false; // flip to true to trace piece/intake-point distance each tick

    private static final float FIELD_HALF_SIZE_M = 1.8288f; // 12 ft / 2
    private static final float WALL_HEIGHT_M = 0.3f;        // illustrative, not sourced from a specific game's real field wall height
    private static final float WALL_THICKNESS_M = 0.02f;

    private final AssetManager assetManager;
    private final Node rootNode;
    private final PhysicsSpace physicsSpace;

    private RigidBodyControl chassisControl;
    boolean driveControllerEnabled=true;
    private RigidBodyControl gamePieceControl;
    private Node gamePieceNode;
    record GamePiece(String id, String type, Node node, RigidBodyControl body,
                     Vector3f initialPosition, com.jme3.math.Quaternion initialRotation) { }
    private final java.util.ArrayList<GamePiece> gamePieces = new java.util.ArrayList<>();
    List<GamePiece> gamePieces() { return List.copyOf(gamePieces); }
    void registerPiece(String id, String type, Node node, RigidBodyControl body) {
        if(gamePieces.stream().anyMatch(p->p.id().equals(id)))throw new IllegalArgumentException("Duplicate game piece: "+id);
        gamePieces.add(new GamePiece(id,type,node,body,body.getPhysicsLocation(),body.getPhysicsRotation()));
    }
    void removePiece(String id){var p=gamePieces.stream().filter(x->x.id().equals(id)).findFirst().orElseThrow();if(physicsSpace.contains(p.body()))physicsSpace.remove(p.body());p.node().removeFromParent();gamePieces.remove(p);}
    void rememberPieceStart(String id) {
        for(int i=0;i<gamePieces.size();i++){var p=gamePieces.get(i);if(p.id().equals(id))gamePieces.set(i,new GamePiece(p.id(),p.type(),p.node(),p.body(),p.body().getPhysicsLocation(),p.body().getPhysicsRotation()));}
    }
    void resetPieces() {
        for(var p:gamePieces) {
            if(!physicsSpace.contains(p.body()))physicsSpace.add(p.body());
            p.body().clearForces();p.body().setPhysicsLocation(p.initialPosition());p.body().setPhysicsRotation(p.initialRotation());
            p.body().setLinearVelocity(Vector3f.ZERO);p.body().setAngularVelocity(Vector3f.ZERO);p.body().activate();
        }
        pieceHeld=false;
        if(flexibleIntake!=null && flexibleIntake.retention!=null)flexibleIntake.retention.reset();
    }
    private boolean pieceHeld = false;
    private physics.MecanumKinematics.ChassisVelocity driveTarget =
        new physics.MecanumKinematics.ChassisVelocity(0, 0, 0);
    private com.jme3.math.Quaternion bodyToRobot = new com.jme3.math.Quaternion();
    private Vector3f robotOriginInBody = new Vector3f();
    private TireDrive tireDrive;
    private DriveContacts driveContacts;
    private FlexibleIntake flexibleIntake;
    private float flexVisualTime;
    private float driveMassKg;
    private float driveYawInertia;
    private double driveResponseTimeS = .10, driveMaxAccelMps2 = 7.8;
    private double yawResponseTimeS = .10, driveMaxYawAccelRadps2 = 20;
    record DriveImpulse(double xNs,double zNs,double yawNms,double availableNs,double availableNms,double normalNs,float dt) { }
    private DriveImpulse driveImpulse=new DriveImpulse(0,0,0,0,0,0,0);
    DriveImpulse driveImpulse(){return driveImpulse;}


    public PhysicsWorld(AssetManager assetManager, Node rootNode, BulletAppState bulletAppState) {
        this(assetManager, rootNode, bulletAppState.getPhysicsSpace());
    }

    PhysicsWorld(AssetManager assetManager, Node rootNode, PhysicsSpace space) {
        this.assetManager = assetManager;
        this.rootNode = rootNode;
        this.physicsSpace = space;
    }

    public void buildFieldBoundary() {
        addStaticBox("floor", new Vector3f(FIELD_HALF_SIZE_M, 0.01f, FIELD_HALF_SIZE_M),
            new Vector3f(0, -0.01f, 0), ColorRGBA.DarkGray);
        // The rendered tile plane already supplies the floor; coincident faces cause z-fighting.
        rootNode.getChild("floor").setCullHint(com.jme3.scene.Spatial.CullHint.Always);

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
    public void buildGamePiece(Vector3f position) { buildGamePiece("practice-torus",position); }

    void buildGamePiece(String id, Vector3f position) {
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
        registerPiece(id,"torus",gamePieceNode,gamePieceControl);
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
            HullCollisionShape shape = new HullCollisionShape(hull);
            shape.setMargin(.002f); // 2 mm contact skin; Bullet's default 4 cm overwhelms FTC mechanisms.
            compound.addChildShape(shape);
        }
        System.out.println("[PHYSICS] V-HACD decomposed game piece into " + hulls.size() + " convex hull(s).");
        return compound;
    }

    public void buildChassis(Node visualNode, double massKg, Vector3f startPosition) {
        CollisionShape shape = new BoxCollisionShape(new Vector3f(0.2286f, 0.1f, 0.2286f));
        buildChassis(visualNode, massKg, startPosition, shape);
    }

    public void buildChassis(Node visualNode, double massKg, Vector3f startPosition, CollisionShape shape) {
        chassisControl = new RigidBodyControl(shape, (float) massKg);
        driveMassKg = (float) massKg;
        driveYawInertia = 1f / chassisControl.getInverseInertiaLocal(null).y;
        chassisControl.setAngularFactor(new Vector3f(0, 1, 0)); // level mecanum constraint
        chassisControl.setFriction(0); // planar rolling resistance is modeled by the drive controller
        chassisControl.setEnableSleep(false);
        visualNode.addControl(chassisControl);
        // Attaching a RigidBodyControl copies the spatial transform into Bullet; place it afterwards.
        chassisControl.setPhysicsLocation(startPosition);
        physicsSpace.add(chassisControl);
        physicsSpace.addTickListener(new PhysicsTickListener() {
            @Override public void prePhysicsTick(PhysicsSpace space, float dt) { applyDriveImpulse(dt); }
            @Override public void physicsTick(PhysicsSpace space, float dt) { }
        });
    }

    /** Sets the target; finite traction impulses are applied on fixed physics ticks. */
    public void driveChassis(physics.MecanumKinematics.ChassisVelocity desiredRobotFrameVelocity, float dtSeconds) {
        driveTarget = desiredRobotFrameVelocity;
    }

    public void configureDrive(double responseTimeS, double maxAccelMps2,
                               double yawResponseTimeS, double maxYawAccelRadps2) {
        for (double value : new double[]{responseTimeS, maxAccelMps2, yawResponseTimeS, maxYawAccelRadps2})
            if (!Double.isFinite(value) || value <= 0) throw new IllegalArgumentException("Drive calibration must be finite and positive");
        this.driveResponseTimeS = responseTimeS;
        this.driveMaxAccelMps2 = maxAccelMps2;
        this.yawResponseTimeS = yawResponseTimeS;
        this.driveMaxYawAccelRadps2 = maxYawAccelRadps2;
    }

    private void applyDriveImpulse(float dt) {
        driveImpulse=new DriveImpulse(0,0,0,0,0,0,dt);
        if (!driveControllerEnabled || dt <= 0 || !chassisControl.isDynamic()) return;
        var support=driveContacts==null?null:driveContacts.refresh();
        if (tireDrive != null) { tireDrive.tick(dt); return; }
        if(support!=null&&support.wheels().isEmpty())return;
        Vector3f desired = getChassisRotation().mult(new Vector3f(
            (float) driveTarget.vx, 0, (float) -driveTarget.vy));
        Vector3f actual = chassisControl.getLinearVelocity();
        desired.y = 0;
        Vector3f error = new Vector3f(desired.x - actual.x, 0, desired.z - actual.z);
        // Exact first-order response avoids explicit-Euler gain explosions at large timesteps.
        float response = (float) -Math.expm1(-dt / driveResponseTimeS);
        Vector3f delta = error.mult(response);
        float maxDelta = (float) driveMaxAccelMps2 * dt;
        if (delta.length() > maxDelta) delta.normalizeLocal().multLocal(maxDelta);
        Vector3f impulse=delta.mult(driveMassKg);
        float yawResponse = (float) -Math.expm1(-dt / yawResponseTimeS);
        float yawDelta = ((float) driveTarget.omega - chassisControl.getAngularVelocity().y) * yawResponse;
        float yawCap = (float) driveMaxYawAccelRadps2 * dt;
        yawDelta = Math.max(-yawCap, Math.min(yawCap, yawDelta));
        float moment=yawDelta*driveYawInertia;
        if(support!=null) {
            double available=support.frictionImpulse(),availableMoment=support.momentImpulse();
            if(availableMoment<=0)moment=0;
            double scale=gripScale(impulse,moment,available,availableMoment);
            impulse.multLocal((float)scale);moment*=scale;
            double rolling=driveContacts.config.rollingResistanceCoefficient()*support.normalImpulse();
            var planar=new Vector3f(actual.x,0,actual.z);float speed=planar.length();
            if(speed>0)impulse.addLocal(planar.mult((float)(-Math.min(rolling,driveMassKg*speed)/speed)));
            // Rolling drag follows the bounded motor request; braking still shares the same grip budget.
            scale=gripScale(impulse,moment,available,availableMoment);
            impulse.multLocal((float)scale);moment*=scale;
        }
        chassisControl.applyCentralImpulse(impulse);
        chassisControl.applyTorqueImpulse(new Vector3f(0, moment, 0));
        driveImpulse=new DriveImpulse(impulse.x,impulse.z,moment,support==null?0:support.frictionImpulse(),support==null?0:support.momentImpulse(),support==null?0:support.normalImpulse(),dt);
    }

    private static double gripScale(Vector3f impulse,float moment,double available,double availableMoment) {
        if(available<=0)return 0;
        double utilization=Math.hypot(impulse.length()/available,availableMoment>0?moment/availableMoment:0);
        return utilization>1?1/utilization:1;
    }

    void setDriveAssemblyProperties(float massKg, float yawInertia) {
        driveMassKg = massKg;
        driveYawInertia = yawInertia;
    }

    void installTires(TireDrive model) { tireDrive = model; }
    void installDriveContacts(DriveContacts model){driveContacts=model;}
    DriveContacts driveContacts(){return driveContacts;}
    TireDrive tireDrive() { return tireDrive; }
    void installFlexibleIntake(FlexibleIntake model) { flexibleIntake=model; }
    FlexibleIntake flexibleIntake() { return flexibleIntake; }
    void addFlexibleYawInertia(float moment) { driveYawInertia += moment; }
    void updateFlexibleVisuals(float dt) {
        flexVisualTime += dt;
        if (flexibleIntake != null && flexVisualTime >= 1f/30) {flexibleIntake.updateVisual();flexVisualTime=0;}
    }
    RigidBodyControl gamePieceBody() { return gamePieceControl; }
    float driveMass() { return driveMassKg; }
    float driveYawInertia() { return driveYawInertia; }

    RigidBodyControl chassisBody() { return chassisControl; }
    PhysicsSpace space() { return physicsSpace; }

    public Vector3f getChassisPosition() { return chassisControl.getPhysicsLocation(); }
    void setRobotFrame(com.jme3.math.Quaternion bodyToRobot, Vector3f robotOriginInBody) {
        this.bodyToRobot = bodyToRobot.clone();
        this.robotOriginInBody = robotOriginInBody.clone();
    }
    /** Joint impulses bypass Bullet angular factors; explicitly lock pitch/roll for articulated chassis. */
    void constrainChassisLevel() {
        chassisControl.setAngularFactor(1);
        var level = new com.jme3.bullet.joints.New6Dof(chassisControl, Vector3f.ZERO,
            getChassisPosition(), bodyToRobot.toRotationMatrix(), new com.jme3.math.Matrix3f(),
            com.jme3.bullet.RotationOrder.YZX);
        for (int dof = 0; dof < 6; dof++) {
            boolean free = dof < 3 || dof == 4;
            level.set(com.jme3.bullet.joints.motors.MotorParam.LowerLimit, dof, free ? 1 : 0);
            level.set(com.jme3.bullet.joints.motors.MotorParam.UpperLimit, dof, free ? -1 : 0);
        }
        physicsSpace.add(level);
    }

    public Vector3f robotPointWorld(Vector3f robotPoint) {
        return getChassisPosition().add(chassisControl.getPhysicsRotation().mult(robotOriginInBody))
            .add(getChassisRotation().mult(robotPoint));
    }
    public com.jme3.math.Quaternion getChassisRotation() { return chassisControl.getPhysicsRotation().mult(bodyToRobot); }
    public Vector3f getChassisAngularVelocity() { return chassisControl.getAngularVelocity(); }

    /**
     * Proximity-trigger intake (per Phase 4's spec: an acceptable simplified model, not full
     * contact dynamics). When active and the game piece is within captureRadius of the
     * intake point, the piece is removed from free physics and parented to the chassis
     * visually; deactivating releases it back into free physics at its held position.
     */
    public void updateIntake(boolean intakeActive, Vector3f intakePointWorld, float captureRadiusM) {
        if (flexibleIntake != null) {
            pieceHeld = gamePieces.stream().anyMatch(p->flexibleIntake.contains(p.body().getPhysicsLocation()));
            return; // Contact mode keeps the game piece in Bullet, including when contained.
        }
        if (gamePieceControl == null) {pieceHeld=false;return;} // BIOBUZZ balls stay physical, including with legacy servo code.

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
            gamePieceControl.setPhysicsLocation(gamePieceNode.getLocalTranslation());
            physicsSpace.add(gamePieceControl);
            gamePieceControl.setLinearVelocity(Vector3f.ZERO);
            pieceHeld = false;
            System.out.println("[PHYSICS] Game piece released.");
        }

        if (pieceHeld) {
            gamePieceControl.setPhysicsLocation(intakePointWorld);
            gamePieceNode.setLocalTranslation(intakePointWorld);
        }
    }

    public boolean isPieceHeld() { return pieceHeld; }
    public Vector3f getGamePiecePosition() {
        if(gamePieceControl!=null)return pieceHeld && flexibleIntake==null ? gamePieceNode.getLocalTranslation() : gamePieceControl.getPhysicsLocation();
        return gamePieces.isEmpty()?null:gamePieces.get(0).body().getPhysicsLocation();
    }

    private Material unshaded(ColorRGBA color) {
        Material m = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        m.setColor("Color", color);
        return m;
    }
}
