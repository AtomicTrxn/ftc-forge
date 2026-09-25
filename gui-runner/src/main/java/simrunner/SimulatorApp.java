package simrunner;

import com.jme3.app.SimpleApplication;
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
import com.jme3.scene.shape.Quad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import physics.ChassisPose;
import physics.MecanumKinematics;
import simcore.ConsoleTelemetry;
import simcore.HardwareMapBuilder;
import simcore.PresetRobotConfig;
import simcore.RobotConfigXml;

import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Path;
import java.util.List;

/**
 * Phase 2: jME orthographic top-down "2D canvas" (per R2 -- jMonkeyEngine, not a separate
 * 2D toolkit, so Phase 4 extends this same renderer instead of rewriting it). Runs a real
 * OpMode via Phase 1's Executor, feeds its motor state through MecanumKinematics each frame,
 * and moves a robot Node according to the resulting field-frame pose.
 *
 * World-frame convention: field lies in the XZ plane (Y=0, jME's default ground plane).
 * ChassisPose's (xMeters, yMeters) map to world (X, -Z); heading maps to a rotation about
 * world +Y. This mapping is this project's own convention, not a physical requirement.
 */
public class SimulatorApp extends SimpleApplication {

    private static final float FIELD_SIZE_M = 3.6576f; // 12 ft
    private static final float TILE_SIZE_M = 0.6096f;   // 24 in
    private static final int TILES_PER_SIDE = 6;

    private final Path projectDir;
    private final String opModeName;

    private HardwareMap hardwareMap;
    private final Gamepad gamepad1 = new Gamepad();
    private final Gamepad gamepad2 = new Gamepad();
    private final ChassisPose pose = new ChassisPose();
    private MecanumKinematics kinematics;
    private Node robotNode;
    private Executor.Session opModeSession;
    private String[] motorNames;

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
        setUpOrthoCamera();
        buildField();
        buildRobot();
        bindGamepadControls();
        try {
            loadRobotAndStartOpMode();
        } catch (Exception e) {
            throw new RuntimeException("Failed to load team project at " + projectDir, e);
        }
    }

    private void setUpOrthoCamera() {
        float aspect = (float) cam.getWidth() / cam.getHeight();
        float halfSize = FIELD_SIZE_M / 2f + 0.5f; // small margin around the field
        cam.setParallelProjection(true);
        cam.setFrustum(-1000, 1000, -aspect * halfSize, aspect * halfSize, halfSize, -halfSize);
        cam.setLocation(new Vector3f(0, 10, 0));
        cam.lookAtDirection(new Vector3f(0, -1, 0), new Vector3f(0, 0, -1));
        flyCam.setEnabled(false); // top-down view is fixed; no free-fly camera in v1
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
        Path classesDir = TeamCodeCompiler.compile(sourceRoot, simConfig.extraClasspath);

        URLClassLoader teamLoader = new URLClassLoader(new URL[]{classesDir.toUri().toURL()}, SimulatorApp.class.getClassLoader());
        List<OpModeDiscovery.DiscoveredOpMode> discovered = OpModeDiscovery.discover(classesDir, teamLoader);
        OpModeDiscovery.DiscoveredOpMode target = discovered.stream()
            .filter(d -> d.className.endsWith("." + opModeName) || d.displayName.equals(opModeName))
            .findFirst()
            .orElseThrow(() -> new IllegalArgumentException("No discovered OpMode matches \"" + opModeName + "\""));

        RobotConfigXml xml = RobotConfigXml.parse(projectDir.resolve(simConfig.robotConfig).toFile());
        PresetRobotConfig preset = PresetRobotConfig.load(projectDir.resolve(simConfig.presetMotors));
        hardwareMap = HardwareMapBuilder.build(xml, preset);

        motorNames = new String[]{"left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive"};
        kinematics = new MecanumKinematics(0.30, 0.35, 2.0);

        Telemetry telemetry = new ConsoleTelemetry();
        opModeSession = Executor.start(target, hardwareMap, telemetry, gamepad1, gamepad2);
        System.out.println("[SIM] Running " + target.displayName + " in the jME renderer...");
    }

    @Override
    public void simpleUpdate(float tpf) {
        if (hardwareMap == null) return;

        double pLF = hardwareMap.get(DcMotorEx.class, motorNames[0]).getPower();
        double pRF = hardwareMap.get(DcMotorEx.class, motorNames[1]).getPower();
        double pLB = hardwareMap.get(DcMotorEx.class, motorNames[2]).getPower();
        double pRB = hardwareMap.get(DcMotorEx.class, motorNames[3]).getPower();

        MecanumKinematics.ChassisVelocity v = kinematics.forward(pLF, pRF, pLB, pRB);
        pose.integrate(v, tpf);

        robotNode.setLocalTranslation((float) pose.xMeters, 0.05f, (float) -pose.yMeters);
        robotNode.setLocalRotation(new Quaternion().fromAngleAxis((float) pose.headingRad, Vector3f.UNIT_Y));

        if (opModeSession != null && !opModeSession.isAlive()) {
            System.out.println("[SIM] OpMode finished. Final pose: " + pose);
            opModeSession = null; // avoid repeated stop() calls across frames
            stop();
        }
    }
}
