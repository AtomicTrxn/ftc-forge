package simrunner;

import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.input.Joystick;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Cylinder;
import com.jme3.scene.shape.Quad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import physics.MecanumKinematics;
import simcore.ConsoleTelemetry;
import simcore.HardwareMapBuilder;
import simcore.PresetRobotConfig;
import simcore.RobotConfigXml;
import simcore.RobotUrdf;
import simcore.SimIMU;

import java.net.URLClassLoader;
import java.nio.file.Path;
import java.util.List;

/**
 * Phase 2/4: jME renderer (per R2 -- jMonkeyEngine, not a separate toolkit). Runs a real
 * OpMode via Phase 1's Executor, feeds its motor state through MecanumKinematics each frame,
 * and (per Phase 4) drives a real Libbulletjme/Minie rigid-body chassis with the resulting
 * target velocity -- Bullet resolves wall/game-piece contact, not wheel-ground traction (R2/R6's
 * Mecanum constraint). Camera is now a 3D perspective view (was Phase 2's orthographic
 * top-down), extending the same scene rather than replacing it.
 *
 * World-frame convention: field lies in the XZ plane (Y=0, jME's default ground plane).
 * Chassis-frame (vx, vy) map to world (X, -Z); heading maps to a rotation about world +Y.
 * This mapping is this project's own convention, not a physical requirement.
 */
public class SimulatorApp extends SimpleApplication {

    private static final float FIELD_SIZE_M = 3.6576f; // 12 ft
    private static final float TILE_SIZE_M = 0.6096f;   // 24 in
    private static final int TILES_PER_SIDE = 6;
    private static final double CHASSIS_MASS_KG = 8.5; // per R6's earlier worked example
    private static final float WHEEL_VISUAL_RADIUS_M = 0.048f;

    private final Path projectDir;
    private final String opModeName;

    private HardwareMap hardwareMap;
    private final Gamepad gamepad1 = new Gamepad();
    private final Gamepad gamepad2 = new Gamepad();
    private MecanumKinematics kinematics;
    private Node robotNode;
    private Geometry[] wheelGeoms;
    private double[] wheelVisualAngleRad;
    private BulletAppState bulletAppState;
    private PhysicsWorld physicsWorld;
    private ImportedRobotScene importedScene;
    private Executor.Session opModeSession;
    private String[] motorNames;
    private double simTimeMs;
    private final double[] wheelRadii = {WHEEL_RADIUS_M, WHEEL_RADIUS_M, WHEEL_RADIUS_M, WHEEL_RADIUS_M};

    public SimulatorApp(Path projectDir, String opModeName) {
        this.projectDir = projectDir;
        this.opModeName = opModeName;
    }

    public static void main(String[] args) throws Exception {
        if (args.length < 2) {
            System.err.println("Usage: SimulatorApp <projectDir> <opModeName>");
            System.exit(2);
        }
        SimulatorApp app = new SimulatorApp(Path.of(args[0]), args[1]);
        app.setShowSettings(false);
        app.start();
    }

    @Override
    public void simpleInitApp() {
        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        physicsWorld = new PhysicsWorld(assetManager, rootNode, bulletAppState);

        setUpPerspectiveCamera();
        buildField();
        physicsWorld.buildFieldBoundary();
        bindGamepadControls();
        try {
            loadRobotAndStartOpMode();
        } catch (Exception e) {
            throw new RuntimeException("Failed to load team project at " + projectDir, e);
        }
    }

    /** Phase 4: swaps Phase 2's orthographic top-down camera for a 3D perspective view showing real depth. */
    private void setUpPerspectiveCamera() {
        cam.setLocation(new Vector3f(0f, 2.2f, 3.2f));
        cam.lookAt(new Vector3f(0.4f, 0f, 0f), Vector3f.UNIT_Y);
        flyCam.setEnabled(false); // fixed 3/4 view in v1; free-fly camera is a future nicety, not required here
    }

    private void buildField() {
        Node field = new Node("field");
        for (int row = 0; row < TILES_PER_SIDE; row++) {
            for (int col = 0; col < TILES_PER_SIDE; col++) {
                Quad quad = new Quad(TILE_SIZE_M, TILE_SIZE_M);
                Geometry tile = new Geometry("tile-" + row + "-" + col, quad);
                Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
                boolean dark = (row + col) % 2 == 0;
                mat.setColor("Color", dark ? new ColorRGBA(0.15f, 0.15f, 0.18f, 1f) : new ColorRGBA(0.6f, 0.1f, 0.1f, 1f));
                tile.setMaterial(mat);
                tile.setLocalRotation(new Quaternion().fromAngleAxis(-FastMath.HALF_PI, Vector3f.UNIT_X));
                float x = -FIELD_SIZE_M / 2f + col * TILE_SIZE_M;
                float z = -FIELD_SIZE_M / 2f + row * TILE_SIZE_M;
                tile.setLocalTranslation(x, 0, z);
                field.attachChild(tile);
            }
        }
        rootNode.attachChild(field);
    }

    private void buildRobot() {
        robotNode = new Node("robot");

        Box body = new Box(0.2286f, 0.05f, 0.2286f); // 18in square footprint (half-extents)
        Geometry bodyGeom = new Geometry("robot-body", body);
        Material bodyMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        bodyMat.setColor("Color", ColorRGBA.Blue);
        bodyGeom.setMaterial(bodyMat);
        bodyGeom.setLocalTranslation(0, 0.1f, 0);
        robotNode.attachChild(bodyGeom);

        // Front-facing marker so heading is visually readable, not just position.
        Box frontMarker = new Box(0.05f, 0.06f, 0.02f);
        Geometry frontGeom = new Geometry("robot-front", frontMarker);
        Material frontMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        frontMat.setColor("Color", ColorRGBA.Yellow);
        frontGeom.setMaterial(frontMat);
        frontGeom.setLocalTranslation(0.2286f, 0.15f, 0);
        robotNode.attachChild(frontGeom);

        // Phase 4: visual-only wheels (per R2/R6 -- no separate physics bodies / no wheel-ground
        // contact; each wheel's spin is driven by the real motor model's encoder state).
        wheelGeoms = new Geometry[4];
        wheelVisualAngleRad = new double[4];
        float[][] wheelOffsets = {{0.18f, 0.15f}, {0.18f, -0.15f}, {-0.18f, 0.15f}, {-0.18f, -0.15f}};
        for (int i = 0; i < 4; i++) {
            Cylinder wheelMesh = new Cylinder(8, 16, WHEEL_VISUAL_RADIUS_M, 0.04f, true);
            Geometry wheelGeom = new Geometry("wheel-" + i, wheelMesh);
            Material wheelMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
            wheelMat.setColor("Color", ColorRGBA.Black);
            wheelGeom.setMaterial(wheelMat);
            wheelGeom.setLocalTranslation(wheelOffsets[i][0], 0, wheelOffsets[i][1]);
            wheelGeom.setLocalRotation(new Quaternion().fromAngleAxis(FastMath.HALF_PI, Vector3f.UNIT_X));
            robotNode.attachChild(wheelGeom);
            wheelGeoms[i] = wheelGeom;
        }

        rootNode.attachChild(robotNode);
    }

    /**
     * Real jME joystick wiring (per R2: jME/LWJGL3 provides gamepad input natively, no
     * separate library). DEVIATION (documented): this sandboxed environment has no physical
     * gamepad attached, so this path could not be exercised end-to-end -- keyboard arrow
     * keys are wired as a manual-driving stand-in, and joystick detection is logged so a
     * real controller would be visible if one were plugged in. See Phase 2's RESULTS.md.
     */
    private void bindGamepadControls() {
        Joystick[] joysticks = inputManager.getJoysticks();
        if (joysticks != null) {
            for (Joystick j : joysticks) {
                System.out.println("[SIM] Detected joystick: " + j.getName());
            }
        } else {
            System.out.println("[SIM] No joysticks detected in this environment.");
        }

        inputManager.addMapping("ManualForward", new KeyTrigger(KeyInput.KEY_UP));
        inputManager.addMapping("ManualBack", new KeyTrigger(KeyInput.KEY_DOWN));
        ActionListener listener = (name, isPressed, tpf) -> {
            if (name.equals("ManualForward")) gamepad1.left_stick_y = isPressed ? -1f : 0f;
            if (name.equals("ManualBack")) gamepad1.left_stick_y = isPressed ? 1f : 0f;
        };
        inputManager.addListener(listener, "ManualForward", "ManualBack");
    }

    private void loadRobotAndStartOpMode() throws Exception {
        SimConfig simConfig = SimConfig.load(projectDir);
        Path sourceRoot = projectDir.resolve(simConfig.sourceRoot);
        Path classesDir = TeamCodeCompiler.compile(projectDir, sourceRoot, simConfig.extraClasspath);

        URLClassLoader teamLoader = TeamCodeCompiler.newClassLoader(projectDir, classesDir, simConfig.extraClasspath);
        List<OpModeDiscovery.DiscoveredOpMode> discovered = OpModeDiscovery.discover(classesDir, teamLoader);
        OpModeDiscovery.DiscoveredOpMode target = discovered.stream()
            .filter(d -> d.className.endsWith("." + opModeName) || d.displayName.equals(opModeName))
            .findFirst()
            .orElseThrow(() -> new IllegalArgumentException("No discovered OpMode matches \"" + opModeName + "\""));

        RobotConfigXml xml = RobotConfigXml.parse(projectDir.resolve(simConfig.robotConfig).toFile());
        PresetRobotConfig preset = PresetRobotConfig.load(projectDir.resolve(simConfig.presetMotors));
        hardwareMap = HardwareMapBuilder.build(xml, preset);
        for (IMU imu : hardwareMap.getAll(IMU.class)) {
            ((SimIMU) imu).setLatencyMs(simConfig.imuLatencyMs);
        }
        double trackWidthM = 0.30, wheelBaseM = 0.35;

        if (simConfig.urdf != null) {
            Path urdfPath = projectDir.resolve(simConfig.urdf);
            RobotUrdf urdf = RobotUrdf.parse(urdfPath);
            if (simConfig.totalMassKg != null) urdf = urdf.withTotalMassKg(simConfig.totalMassKg);
            urdf.validateHardwareMap(hardwareMap);
            importedScene = new ImportedRobotScene(urdf, urdfPath, hardwareMap, assetManager, simConfig.vhacdMaxHulls);
            robotNode = importedScene.root;
            rootNode.attachChild(robotNode);
            physicsWorld.buildChassis(robotNode, urdf.totalMassKg(), new Vector3f(0, 0.1f, 0),
                importedScene.chassisShape());
            physicsWorld.setChassisInertia(importedScene.inertiaDiagonal());
            System.out.println("[IMPORT] Physics chassis from " + urdf.name + ", mass=" + urdf.totalMassKg() + "kg");
            double[] wheelX = new double[4], wheelY = new double[4];
            boolean[] wheelFound = new boolean[4];
            String[] drives = {"left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive"};
            for (RobotUrdf.Transmission tx : urdf.transmissions.values()) {
                RobotUrdf.Joint joint = urdf.joints.get(tx.joint());
                if (!joint.type().equals("continuous")) continue;
                for (RobotUrdf.Actuator actuator : tx.actuators()) {
                    for (int i = 0; i < 4; i++) {
                        if (!actuator.name().equals(drives[i])) continue;
                        if (joint.parent().equals(urdf.rootLink)) {
                            wheelX[i] = joint.origin().xyz()[0];
                            wheelY[i] = joint.origin().xyz()[1];
                            wheelFound[i] = true;
                        }
                        for (RobotUrdf.Collision c : urdf.links.get(joint.child()).collisions()) {
                            if (c.geometry().kind().equals("cylinder")) wheelRadii[i] = c.geometry().dimensions()[0];
                        }
                    }
                }
            }
            if (wheelFound[0] && wheelFound[1] && wheelFound[2] && wheelFound[3]) {
                trackWidthM = (Math.abs(wheelY[0] - wheelY[1]) + Math.abs(wheelY[2] - wheelY[3])) / 2;
                wheelBaseM = (Math.abs(wheelX[0] - wheelX[2]) + Math.abs(wheelX[1] - wheelX[3])) / 2;
                if (trackWidthM <= 0 || wheelBaseM <= 0)
                    throw new IllegalArgumentException("Imported drive-wheel layout has zero track width or wheelbase");
                System.out.println("[IMPORT] Mecanum track=" + trackWidthM + "m wheelbase=" + wheelBaseM + "m");
            } else {
                System.out.println("[WARN] Could not derive all four drive-wheel positions from chassis-child joints; using preset kinematics dimensions.");
            }
        } else {
            buildRobot();
            physicsWorld.buildChassis(robotNode, CHASSIS_MASS_KG, new Vector3f(0, 0.1f, 0));
        }
        physicsWorld.buildGamePiece(new Vector3f(0.8f, 0.05f, 0));

        motorNames = new String[]{"left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive"};
        kinematics = new MecanumKinematics(trackWidthM, wheelBaseM, 2.0);

        Telemetry telemetry = new ConsoleTelemetry();
        opModeSession = Executor.start(target, hardwareMap, telemetry, gamepad1, gamepad2);
        System.out.println("[SIM] Running " + target.displayName + " in the jME renderer...");
    }

    @Override
    public void simpleUpdate(float tpf) {
        if (hardwareMap == null) return;

        // Real motor angular velocities (R4's torque/speed model), converted to logical
        // (commanded-sign) wheel speed -- see wheelLinearSpeed's own javadoc for why the
        // Direction correction matters. Also spins each wheel's visual mesh independently.
        DcMotorEx[] driveMotors = {
            hardwareMap.get(DcMotorEx.class, motorNames[0]), hardwareMap.get(DcMotorEx.class, motorNames[1]),
            hardwareMap.get(DcMotorEx.class, motorNames[2]), hardwareMap.get(DcMotorEx.class, motorNames[3])
        };
        double[] wheelSpeeds = new double[4];
        for (int i = 0; i < 4; i++) {
            wheelSpeeds[i] = wheelLinearSpeed(driveMotors[i], wheelRadii[i]);
            if (importedScene == null) {
                wheelVisualAngleRad[i] += (wheelSpeeds[i] / WHEEL_VISUAL_RADIUS_M) * tpf;
                wheelGeoms[i].setLocalRotation(
                    new Quaternion().fromAngleAxis(FastMath.HALF_PI, Vector3f.UNIT_X)
                        .mult(new Quaternion().fromAngleAxis((float) wheelVisualAngleRad[i], Vector3f.UNIT_Z)));
            }
        }
        if (importedScene != null) {
            importedScene.update();
            physicsWorld.setChassisInertia(importedScene.inertiaDiagonal());
        }

        MecanumKinematics.ChassisVelocity v = kinematics.forwardFromWheelSpeeds(
            wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);

        // Phase 4: drive the real rigid-body chassis with target velocity from the Mecanum
        // constraint instead of directly integrating a kinematic pose -- Bullet's
        // RigidBodyControl on robotNode syncs its transform from physics automatically, so
        // there's no manual setLocalTranslation/setLocalRotation here anymore.
        physicsWorld.driveChassis(v, tpf);

        simTimeMs += tpf * 1000.0;
        Vector3f heading = physicsWorld.getChassisRotation().mult(Vector3f.UNIT_X);
        double yawRad = Math.atan2(-heading.z, heading.x);
        double yawRateRadS = physicsWorld.getChassisAngularVelocity().y;
        for (IMU imu : hardwareMap.getAll(IMU.class)) {
            ((SimIMU) imu).update(yawRad, yawRateRadS, Math.round(simTimeMs));
        }

        // Intake: "claw" servo position > 0.5 means active, per this phase's sample OpMode.
        Servo claw = hardwareMap.tryGet(Servo.class, "claw");
        boolean intakeActive = claw != null && claw.getPosition() > 0.5;
        Vector3f chassisPos = physicsWorld.getChassisPosition();
        Vector3f forward = physicsWorld.getChassisRotation().mult(new Vector3f(1, 0, 0));
        Vector3f intakePoint = chassisPos.add(forward.mult(0.35f));
        physicsWorld.updateIntake(intakeActive, intakePoint, 0.15f);

        if (opModeSession != null && !opModeSession.isAlive()) {
            System.out.println("[SIM] OpMode finished. Final chassis position: " + physicsWorld.getChassisPosition()
                + " gamePieceHeld=" + physicsWorld.isPieceHeld());
            opModeSession = null; // avoid repeated stop() calls across frames
            stop();
        }
    }

    private static final double WHEEL_RADIUS_M = 0.048; // goBILDA 96mm mecanum wheel (common FTC drivetrain wheel)

    /**
     * Converts a motor's real physical angular velocity into the "logical" (commanded-sign)
     * wheel speed the kinematics formula expects. This distinction matters and is easy to get
     * wrong: DcMotor.Direction.REVERSE exists specifically so a team's own code can use
     * consistent +power-means-forward semantics despite physically mirrored motor mounting on
     * opposite drivetrain sides -- the kinematics solver's sign convention is defined in terms
     * of that same commanded/logical intent, not raw physical rotation. Using getOmegaRadS()
     * directly (without undoing the Direction flip) silently breaks kinematics for any wheel
     * with Direction.REVERSE set -- caught by an actual renderer run producing a stuck,
     * spinning-in-place robot instead of driving, not assumed correct in advance.
     */
    private double wheelLinearSpeed(DcMotorEx motor, double radiusM) {
        if (motor instanceof simcore.SimDcMotorEx) {
            double physicalOmega = ((simcore.SimDcMotorEx) motor).getOmegaRadS();
            double logicalOmega = motor.getDirection() == DcMotorSimple.Direction.FORWARD ? physicalOmega : -physicalOmega;
            return logicalOmega * radiusM;
        }
        return motor.getPower() * kinematics.maxWheelSpeedMetersPerSecond; // fallback, shouldn't hit in this simulator
    }
}
