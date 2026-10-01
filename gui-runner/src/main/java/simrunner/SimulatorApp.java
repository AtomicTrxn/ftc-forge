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
 * OpMode via Phase 1's Executor, feeds its motor state through the configured drive kinematics each frame,
 * and (per Phase 4) drives a real Libbulletjme/Minie rigid-body chassis with the resulting
 * target velocity -- Bullet resolves wall/game-piece contact with aggregate wheel traction (R2/R6's
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
    private DifferentialDriveConfig differential;
    private MotorIntakeConfig motorIntake;
    private Node robotNode;
    private Geometry[] wheelGeoms;
    private double[] wheelVisualAngleRad;
    private BulletAppState bulletAppState;
    private PhysicsWorld physicsWorld;
    private ImportedRobotScene importedScene;
    private ArticulatedRobot articulated;
    private Executor.Session opModeSession;
    private String[] motorNames;
    private double simTimeMs;
    private Path screenshotPath;
    private com.jme3.app.state.ScreenshotAppState screenshot;
    private int finishingFrames;
    private SimConfig simConfig;
    private String fieldOverride,modeOverride,pieceSetOverride;
    private boolean previewOnly;
    private ImportedFieldScene fieldScene;
    private ModelFieldScene modelFieldScene;
    private com.jme3.font.BitmapText fieldStatus;
    private double renderSeconds;private int renderFrames;
    private int previewFrames;
    private double lastPerfReport;
    private int physicsOverBudgetFrames;private float maxRuntimeFrameMs;
    private boolean collisionReview, collisionOnly, cadVisible=true;
    private Path collisionReport;
    private CollisionOverlay collisionOverlay;
    private final double[] wheelRadii = {WHEEL_RADIUS_M, WHEEL_RADIUS_M, WHEEL_RADIUS_M, WHEEL_RADIUS_M};

    public SimulatorApp(Path projectDir, String opModeName) {
        this.projectDir = projectDir;
        this.opModeName = opModeName;
    }

    public static void main(String[] args) throws Exception {
        if (args.length < 2) {
            System.err.println("Usage: SimulatorApp <projectDir> <opModeName> [screenshot.png] [--field generic|field.json] [--mode field-only|game-pieces] [--piece-set biobuzz|torus] [--preview|--collision-review|--collisions-only] [--collision-report report.json]");
            System.exit(2);
        }
        SimulatorApp app = new SimulatorApp(Path.of(args[0]), args[1]);
        for(int i=2;i<args.length;i++) {
            if(java.util.Set.of("--field","--mode","--piece-set","--collision-report").contains(args[i]) && i+1==args.length)throw new IllegalArgumentException("Missing value for "+args[i]);
            switch(args[i]) {
                case "--field" -> app.fieldOverride=args[++i];
                case "--mode" -> app.modeOverride=args[++i];
                case "--piece-set" -> app.pieceSetOverride=args[++i];
                case "--preview" -> app.previewOnly=true;
                case "--collision-review" -> {app.collisionReview=true;app.previewOnly=true;}
                case "--collisions-only" -> {app.collisionOnly=true;app.collisionReview=true;app.previewOnly=true;}
                case "--collision-report" -> app.collisionReport=Path.of(args[++i]).toAbsolutePath();
                default -> {if(args[i].startsWith("--")||app.screenshotPath!=null)throw new IllegalArgumentException("Unknown/duplicate argument: "+args[i]);app.screenshotPath=Path.of(args[i]).toAbsolutePath();}
            }
        }
        var settings=new com.jme3.system.AppSettings(true);settings.setTitle("FTC Forge — field simulator");settings.setResolution(1280,800);app.setSettings(settings);
        app.setPauseOnLostFocus(false);
        app.setShowSettings(false);
        app.start();
    }

    @Override
    public void simpleInitApp() {
        rootNode.addLight(new com.jme3.light.AmbientLight(new ColorRGBA(.3f, .3f, .3f, 1)));
        rootNode.addLight(new com.jme3.light.DirectionalLight(new Vector3f(-1, -2, -1).normalizeLocal(),new ColorRGBA(.65f,.65f,.65f,1)));
        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        physicsWorld = new PhysicsWorld(assetManager, rootNode, bulletAppState);

        setUpPerspectiveCamera();
        bindGamepadControls();
        try {
            simConfig=SimConfig.load(projectDir);
            var f=simConfig.field;
            if(pieceSetOverride!=null)f=new FieldConfig(f.source(),f.mode(),f.packagePath(),pieceSetOverride,f.fullDetail(),f.friction(),f.restitution());
            if(fieldOverride!=null)f=f.withSource(fieldOverride);
            if(modeOverride!=null)f=f.withMode(modeOverride);
            simConfig.field=f.validated();
            if(f.source().equals("imported")) {
                Path fieldPath=projectDir.resolve(f.packagePath());
                if(fieldPath.getFileName().toString().equals("profile.json")) {
                    var profile=new ModelProfile(fieldPath,true);if(!profile.kind.equals("field"))throw new IllegalArgumentException("Select a field profile");
                    if(profile.legacyField())fieldScene=new ImportedFieldScene(new FieldPackage(profile.artifact("field")),f,physicsWorld,rootNode,assetManager);
                    else modelFieldScene=new ModelFieldScene(profile,f.hasPieces(),physicsWorld,rootNode,assetManager,simConfig.scenePieces);
                } else fieldScene=new ImportedFieldScene(new FieldPackage(fieldPath),f,physicsWorld,rootNode,assetManager);
                if(simConfig.robotStart==null)simConfig.robotStart=new Vector3f(-1.2f,(float)simConfig.startHeightM,1.2f);
            } else {
                buildField();physicsWorld.buildFieldBoundary();
                if(f.hasPieces()&&!f.usesTorus())buildPracticeBalls();
                System.out.println("[FIELD] Generic | 3.6576 x 3.6576 m | units=m scale=1 | "+f.mode()+" | "+physicsWorld.gamePieces().size()+" balls");
            }
            physicsWorld.space().setAccuracy(1f/120);
            physicsWorld.space().setMaxSubSteps(32);
            loadRobotAndStartOpMode();
            collisionOverlay=new CollisionOverlay(physicsWorld.space(),rootNode,assetManager);
            collisionOverlay.setVisible(collisionReview);collisionOverlay.update();
            if(collisionReview)System.out.println("[COLLISION REVIEW] native bodies="+physicsWorld.space().countRigidBodies()+" rendered shapes="+collisionOverlay.shapeCount()+" | TeamCode not started");
            inputManager.addMapping("CollisionOverlay",new KeyTrigger(KeyInput.KEY_C));
            inputManager.addMapping("CollisionCad",new KeyTrigger(KeyInput.KEY_V));
            inputManager.addListener((ActionListener)(name,pressed,tpf)->{if(pressed){if(name.equals("CollisionOverlay"))collisionOverlay.setVisible(!collisionOverlay.visible);else {cadVisible=!cadVisible;setCadVisible(cadVisible);}}},"CollisionOverlay","CollisionCad");
            if(collisionOnly){cadVisible=false;setCadVisible(false);}
            timer.reset();
            fieldStatus=new com.jme3.font.BitmapText(guiFont);fieldStatus.setText((modelFieldScene!=null?modelFieldScene.profile.name:fieldScene==null?"Generic field":fieldScene.field.name)+" | "+f.mode()+" | "+physicsWorld.gamePieces().size()+" pieces\nMeters at scale 1 | drag: orbit | scroll: zoom | R: reset field"+(previewOnly?" | preview":""));
            fieldStatus.setLocalTranslation(15,cam.getHeight()-15,0);guiNode.attachChild(fieldStatus);setDisplayStatView(false);
            fieldStatus.setText(fieldStatus.getText()+"\nC: collision overlay (cyan: dynamic, orange: static) | V: CAD visibility"+(collisionReview?" | coverage is not accuracy certification":""));
            inputManager.addMapping("ResetField",new KeyTrigger(KeyInput.KEY_R));inputManager.addListener((ActionListener)(name,pressed,tpf)->{if(pressed){if(fieldScene!=null)fieldScene.reset();else if(modelFieldScene!=null){modelFieldScene.reset();physicsWorld.resetPieces();}else physicsWorld.resetPieces();}},"ResetField");
            if (screenshotPath != null) {
                java.nio.file.Files.createDirectories(screenshotPath.getParent());
                String name = screenshotPath.getFileName().toString();
                if (!name.endsWith(".png")) throw new IllegalArgumentException("Screenshot must end in .png");
                screenshot = new com.jme3.app.state.ScreenshotAppState(screenshotPath.getParent() + "/", name.substring(0, name.length() - 4));
                screenshot.setIsNumbered(false);
                stateManager.attach(screenshot);
            }
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
                float z = -FIELD_SIZE_M / 2f + (row + 1) * TILE_SIZE_M;
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
        inputManager.addMapping("ManualLeft", new KeyTrigger(KeyInput.KEY_LEFT));
        inputManager.addMapping("ManualRight", new KeyTrigger(KeyInput.KEY_RIGHT));
        inputManager.addMapping("ManualIntake", new KeyTrigger(KeyInput.KEY_SPACE));
        inputManager.addMapping("ManualEject", new KeyTrigger(KeyInput.KEY_E));
        ActionListener listener = (name, isPressed, tpf) -> {
            if (name.equals("ManualForward")) gamepad1.left_stick_y = isPressed ? -1f : 0f;
            if (name.equals("ManualBack")) gamepad1.left_stick_y = isPressed ? 1f : 0f;
            if (name.equals("ManualLeft")) gamepad1.right_stick_x = isPressed ? -1f : 0f;
            if (name.equals("ManualRight")) gamepad1.right_stick_x = isPressed ? 1f : 0f;
            if (name.equals("ManualIntake")) gamepad1.a = isPressed;
            if (name.equals("ManualEject")) gamepad1.b = isPressed;
        };
        inputManager.addListener(listener, "ManualForward", "ManualBack", "ManualLeft", "ManualRight", "ManualIntake", "ManualEject");
    }

    private void loadRobotAndStartOpMode() throws Exception {
        differential = simConfig.drive;
        motorIntake = simConfig.intake;
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
        if (simConfig.calibration != null) {
            CalibrationProfile profile = CalibrationProfile.load(projectDir.resolve(simConfig.calibration));
            profile.applyHardware(hardwareMap);
            profile.applyDrive(physicsWorld);
            System.out.println("[CALIBRATION] Loaded " + simConfig.calibration);
        }
        for (IMU imu : hardwareMap.getAll(IMU.class)) {
            ((SimIMU) imu).setLatencyMs(simConfig.imuLatencyMs);
        }
        motorNames = differential == null ? new String[]{"left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive"}
            : differential.motorNames().toArray(String[]::new);
        for (String name : motorNames) hardwareMap.get(simcore.SimDcMotorEx.class, name);
        if (motorIntake != null) hardwareMap.get(simcore.SimDcMotorEx.class, motorIntake.motor());
        double trackWidthM = 0.30, wheelBaseM = 0.35;

        if (simConfig.urdf != null) {
            Path urdfPath = projectDir.resolve(simConfig.urdf);
            RobotUrdf urdf = RobotUrdf.parse(urdfPath);
            if (simConfig.totalMassKg != null) urdf = urdf.withTotalMassKg(simConfig.totalMassKg);
            urdf.validateHardwareMap(hardwareMap);
            importedScene = new ImportedRobotScene(urdf, urdfPath, hardwareMap, assetManager, simConfig.vhacdMaxHulls, java.util.Set.of(motorNames),simConfig.field.fullDetail()?0:.0005f);
            System.out.println("[ROBOT VISUAL] unique triangles "+importedScene.sourceVisualTriangles+" -> "+importedScene.preparedVisualTriangles+"; grid="+(simConfig.field.fullDetail()?0:.0005)+"m, vertex movement <=0.4331mm; collision and inertia use original CAD.");
            if(simConfig.robotProfile!=null)simConfig.robotProfile.configure(importedScene);
            importedScene.tireContacts = simConfig.tires != null;
            importedScene.flexibleIntake = simConfig.flexibleIntake;
            importedScene.collisionOmissions=simConfig.collisionOmissions;
            var collisionAudit=CollisionAudit.inspect(importedScene);
            System.out.println("[COLLISION AUDIT] "+collisionAudit.summary());
            if(collisionReport!=null)collisionAudit.write(collisionReport,urdfPath);
            collisionAudit.requireUsable();
            Vector3f start=simConfig.robotStart==null?new Vector3f(0,(float)simConfig.startHeightM,0):simConfig.robotStart;
            validateStart(start);
            articulated = new ArticulatedRobot(importedScene, physicsWorld, rootNode, start, simConfig.servoPhysics,new Quaternion().fromAngleAxis(simConfig.robotYawRad,Vector3f.UNIT_Y));
            robotNode = articulated.chassisNode;
            if (simConfig.tires != null) {
                physicsWorld.installTires(new TireDrive(physicsWorld, hardwareMap, importedScene, differential, simConfig.tires));
                physicsWorld.space().setAccuracy(1f / 120);
            }
            if (simConfig.flexibleIntake != null) {
                physicsWorld.installFlexibleIntake(new FlexibleIntake(physicsWorld, importedScene, articulated, simConfig.flexibleIntake));
                if (simConfig.torusRetention != null && simConfig.field.usesTorus()) physicsWorld.flexibleIntake().installRetention(articulated, hardwareMap, simConfig.intake, simConfig.torusRetention);
                else if(simConfig.torusRetention!=null)System.out.println("[RETENTION] Torus profile inactive for "+simConfig.field.mode()+" / "+simConfig.field.pieceSet()+"; balls use native contacts.");
                physicsWorld.space().setAccuracy(1f / 480);
                physicsWorld.space().setMaxSubSteps(64);
            }
            System.out.println("[IMPORT] Physics chassis from " + urdf.name + ", mass=" + urdf.totalMassKg() + "kg");
            if (differential == null) {
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
            }
        } else {
            buildRobot();
            Vector3f start=simConfig.robotStart==null?new Vector3f(0,.1f,0):simConfig.robotStart;validateStart(start);
            physicsWorld.buildChassis(robotNode, CHASSIS_MASS_KG,start);
            physicsWorld.chassisBody().setPhysicsRotation(new Quaternion().fromAngleAxis(simConfig.robotYawRad,Vector3f.UNIT_Y));
        }
        {
            cam.setFrustumPerspective(45, (float) cam.getWidth() / cam.getHeight(), .01f, 30f);
            Node cameraTarget=robotNode;
            if(!simConfig.field.usesTorus()) {cameraTarget=new Node("field-camera-target");cameraTarget.setLocalTranslation(0,fieldScene==null?0:.6f,0);rootNode.attachChild(cameraTarget);}
            var orbit = new com.jme3.input.ChaseCamera(cam, cameraTarget, inputManager);
            orbit.setDefaultDistance(simConfig.field.usesTorus() && importedScene!=null?1.2f:6.5f);
            orbit.setMinDistance(.35f);
            orbit.setMaxDistance(10);
            orbit.setDefaultHorizontalRotation(.75f);
            orbit.setDefaultVerticalRotation(.65f);
            orbit.setTrailingEnabled(false);
            orbit.setDragToRotate(true);
            setDisplayStatView(false);
        }
        if(simConfig.field.usesTorus()) {
            var start=simConfig.gamePieceStart==null?new Vector3f(.8f,simConfig.flexibleIntake==null?.05f:.034f,0):simConfig.gamePieceStart;
            if(Math.abs(start.x)+.12f>1.8288f||Math.abs(start.z)+.12f>1.8288f)throw new IllegalArgumentException("Practice torus starts outside generic field");
            physicsWorld.buildGamePiece(start);
            if(simConfig.flexibleIntake!=null)physicsWorld.gamePieceBody().setPhysicsRotation(new Quaternion().fromAngleAxis(FastMath.HALF_PI,Vector3f.UNIT_X));
            physicsWorld.rememberPieceStart("practice-torus");
        }

        if(simConfig.field.hasPieces()){if(fieldScene!=null)SceneProfile.legacyPieces(simConfig.scenePieces,fieldScene,physicsWorld);if(modelFieldScene==null)SceneProfile.placePieces(simConfig.scenePieces,physicsWorld);}
        kinematics = new MecanumKinematics(trackWidthM, wheelBaseM, 2.0);
        validateInitialContacts();

        Telemetry telemetry = new ConsoleTelemetry();
        if(!previewOnly)opModeSession = Executor.start(target, hardwareMap, telemetry, gamepad1, gamepad2);
        System.out.println(previewOnly?"[SIM] Physical preview of "+target.displayName+" | TeamCode not started":"[SIM] Running " + target.displayName + " in the jME renderer...");
    }

    @Override
    public void simpleUpdate(float tpf) {
        if(collisionOverlay!=null)collisionOverlay.update();
        if(hardwareMap!=null){renderSeconds+=tpf;renderFrames++;if(renderFrames>5){maxRuntimeFrameMs=Math.max(maxRuntimeFrameMs,tpf*1000);if(tpf>physicsWorld.space().getAccuracy()*physicsWorld.space().maxSubSteps())physicsOverBudgetFrames++;}if(renderSeconds-lastPerfReport>=2){lastPerfReport=renderSeconds;System.out.printf(java.util.Locale.ROOT,"[PERF] mean FPS=%.1f bodies=%d pieces=%d%n",renderFrames/renderSeconds,physicsWorld.space().countRigidBodies(),physicsWorld.gamePieces().size());}}
        if(previewOnly && screenshot!=null && ++previewFrames==30){screenshot.takeScreenshot();finishingFrames=10;}
        if (finishingFrames > 0) {
            if (--finishingFrames == 0) stop();
            return;
        }
        if (hardwareMap == null) return;

        // Real motor angular velocities (R4's torque/speed model), converted to logical
        // (commanded-sign) wheel speed -- see wheelLinearSpeed's own javadoc for why the
        // Direction correction matters. Also spins each wheel's visual mesh independently.
        double[] wheelSpeeds = new double[4];
        if (differential == null) {
            for (int i = 0; i < 4; i++) {
                wheelSpeeds[i] = wheelLinearSpeed(hardwareMap.get(DcMotorEx.class, motorNames[i]), wheelRadii[i]);
            }
        } else if (importedScene == null) {
            wheelSpeeds[0] = wheelSpeeds[2] = hardwareMap.get(simcore.SimDcMotorEx.class, differential.leftMotor()).getOmegaRadS()
                * differential.leftShaftSign() * differential.wheelRadiusM();
            wheelSpeeds[1] = wheelSpeeds[3] = hardwareMap.get(simcore.SimDcMotorEx.class, differential.rightMotor()).getOmegaRadS()
                * differential.rightShaftSign() * differential.wheelRadiusM();
        }
        if (importedScene == null) {
            for (int i = 0; i < 4; i++) {
                wheelVisualAngleRad[i] += (wheelSpeeds[i] / WHEEL_VISUAL_RADIUS_M) * tpf;
                wheelGeoms[i].setLocalRotation(new Quaternion().fromAngleAxis(FastMath.HALF_PI, Vector3f.UNIT_X)
                    .mult(new Quaternion().fromAngleAxis((float) wheelVisualAngleRad[i], Vector3f.UNIT_Z)));
            }
        }

        if (importedScene != null) {
            importedScene.update();
            physicsWorld.updateFlexibleVisuals(tpf);
        }

        MecanumKinematics.ChassisVelocity v;
        if (differential != null) v = differential.velocity(hardwareMap);
        else v = kinematics.forwardFromWheelSpeeds(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);

        // Drive the real rigid-body chassis with target velocity from the configured drive
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

        // Legacy servo intake defaults; an explicit motor intake uses physical shaft speed.
        Servo claw = hardwareMap.tryGet(Servo.class, "claw");
        boolean intakeActive = claw != null && claw.getPosition() > 0.5;
        Vector3f chassisPos = physicsWorld.getChassisPosition();
        Vector3f forward = physicsWorld.getChassisRotation().mult(new Vector3f(1, 0, 0));
        Vector3f intakePoint = chassisPos.add(forward.mult(0.35f));
        if (motorIntake != null) {
            intakeActive = motorIntake.active(hardwareMap);
            intakePoint = physicsWorld.robotPointWorld(motorIntake.point());
        }
        physicsWorld.updateIntake(intakeActive, intakePoint, motorIntake == null ? .15f : motorIntake.captureRadiusM());

        if (opModeSession != null && !opModeSession.isAlive()) {
            if (opModeSession.result.failure != null)
                throw new RuntimeException("OpMode failed", opModeSession.result.failure);
            System.out.println("[SIM] OpMode finished. Final chassis position: " + physicsWorld.getChassisPosition()
                + " yaw=" + yawRad + (physicsWorld.flexibleIntake()==null ? " gamePieceHeld=" : " gamePieceContained=") + physicsWorld.isPieceHeld()
                + " piece=" + physicsWorld.getGamePiecePosition() + " intake=" + intakePoint);
            System.out.printf(java.util.Locale.ROOT,"[PERF] run mean FPS=%.1f frames=%d wallFramesSeconds=%.2f pieces=%d heapUsedMiB=%.1f%n",renderFrames/renderSeconds,renderFrames,renderSeconds,physicsWorld.gamePieces().size(),(Runtime.getRuntime().totalMemory()-Runtime.getRuntime().freeMemory())/1048576.);
            System.out.printf(java.util.Locale.ROOT,"[PERF] physicsOverBudgetFrames=%d maxRuntimeFrameMs=%.2f fixedStepMs=%.3f%n",physicsOverBudgetFrames,maxRuntimeFrameMs,physicsWorld.space().getAccuracy()*1000);
            if (physicsWorld.flexibleIntake() != null && physicsWorld.flexibleIntake().retention != null) {
                var grip=physicsWorld.flexibleIntake().retention;
                System.out.println("[RETENTION] state="+grip.state()+" acquisitions="+grip.acquisitions+" releases="+grip.releases+" peakForceN="+grip.peakForceN+" peakTorqueNm="+grip.peakTorqueNm+" peakShaftLoadNm="+grip.peakLoadNm);
            }
            if (physicsWorld.flexibleIntake() != null) System.out.println("[FLEX] finalBendRad="+physicsWorld.flexibleIntake().maxDeflectionRad());
            if (physicsWorld.tireDrive() != null) System.out.println("[TIRES] " + physicsWorld.tireDrive().states());
            if (articulated != null) System.out.println("[SIM] Final physical joints: " + articulated.jointPositions());
            opModeSession = null; // avoid repeated stop() calls across frames
            if (screenshot == null) stop();
            else {
                screenshot.takeScreenshot();
                finishingFrames = 10;
            }
        }
    }

    private void validateStart(Vector3f start) {
        var half=modelFieldScene!=null?modelFieldScene.halfExtents:fieldScene==null?new Vector3f(1.8288f,0,1.8288f):fieldScene.field.halfExtents;
        float minX=-.2286f,maxX=.2286f,minZ=-.2286f,maxZ=.2286f,minY=-.1f;
        if(importedScene!=null){importedScene.root.updateGeometricState();if(importedScene.root.getWorldBound() instanceof com.jme3.bounding.BoundingBox box) {
            minX=Float.POSITIVE_INFINITY;maxX=Float.NEGATIVE_INFINITY;minZ=Float.POSITIVE_INFINITY;maxZ=Float.NEGATIVE_INFINITY;minY=box.getCenter().y-box.getYExtent();
            var yaw=new Quaternion().fromAngleAxis(simConfig.robotYawRad,Vector3f.UNIT_Y);
            for(float x:new float[]{box.getCenter().x-box.getXExtent(),box.getCenter().x+box.getXExtent()})for(float z:new float[]{box.getCenter().z-box.getZExtent(),box.getCenter().z+box.getZExtent()}){var p=yaw.mult(new Vector3f(x,0,z));minX=Math.min(minX,p.x);maxX=Math.max(maxX,p.x);minZ=Math.min(minZ,p.z);maxZ=Math.max(maxZ,p.z);}
        }}
        if(!Float.isFinite(start.x)||!Float.isFinite(start.y)||!Float.isFinite(start.z)||start.x+minX<-half.x||start.x+maxX>half.x||start.z+minZ<-half.z||start.z+maxZ>half.z||start.y+minY<(modelFieldScene!=null?(float)FieldPackage.num(modelFieldScene.profile.parameters,"floor_top_m"):fieldScene!=null&&fieldScene.field.modelParameters.containsKey("floor_top_m")?(float)FieldPackage.num(fieldScene.field.modelParameters,"floor_top_m"):0)-.004f)
            throw new IllegalArgumentException("Robot start footprint must be inside the selected field");
        if(fieldScene!=null && start.x+maxX>-.65f && start.x+minX<.65f && start.z+maxZ>-.5f && start.z+minZ<.5f)throw new IllegalArgumentException("Robot start overlaps the HIVE/frame envelope; choose robot_start_xyz_m outside it");
    }
    private void setCadVisible(boolean visible) {
        // Hide only render geometry: body controls and native shapes remain active.
        for(var child:rootNode.getChildren())if(child!=collisionOverlay.root)
            child.setCullHint(visible?com.jme3.scene.Spatial.CullHint.Inherit:com.jme3.scene.Spatial.CullHint.Always);
    }
    private void validateInitialContacts() {
        var environment=new java.util.HashSet<Long>();
        if(fieldScene!=null){for(var body:fieldScene.fixed)environment.add(body.nativeId());for(var hive:fieldScene.hives)environment.add(hive.body().nativeId());}
        if(modelFieldScene!=null)for(var body:modelFieldScene.bodies)environment.add(body.nativeId());
        for(var piece:physicsWorld.gamePieces())environment.add(piece.body().nativeId());
        var half=modelFieldScene!=null?modelFieldScene.halfExtents:fieldScene!=null?fieldScene.field.halfExtents:new Vector3f(1.8288f,0,1.8288f);
        for(var piece:physicsWorld.gamePieces()) {
            var p=piece.body().getPhysicsLocation();
            if(p.y<0||Math.abs(p.x)>half.x||Math.abs(p.z)>half.z)throw new IllegalArgumentException("Game piece starts outside the selected field; edit the saved scene: "+piece.id());
            physicsWorld.space().contactTest(piece.body(),event->{if(event.getDistance1()<-.004f)throw new IllegalArgumentException("Game-piece start penetrates an obstacle or another piece; edit the saved scene: "+piece.id());});
        }
        for(var body:physicsWorld.space().getRigidBodyList())if(body.isDynamic()&&!environment.contains(body.nativeId())) {
            physicsWorld.space().contactTest(body,event->{
                long other=event.getObjectA().nativeId()==body.nativeId()?event.getObjectB().nativeId():event.getObjectA().nativeId();
                if(environment.contains(other)&&event.getDistance1()<-.004f)throw new IllegalArgumentException("Robot start penetrates a field obstacle or game piece; choose another robot_start_xyz_m");
            });
        }
    }
    private void buildPracticeBalls() {
        for(int i=0;i<6;i++) {
            float radius=i<2?.03556f:.04597f;String type=i<2?"pollen":i<4?"red_nectar":"blue_nectar";
            Node node=new Node("practice-"+type+"-"+i);var sphere=new com.jme3.scene.shape.Sphere(12,24,radius);var g=new Geometry(node.getName(),sphere);
            var m=new Material(assetManager,"Common/MatDefs/Misc/Unshaded.j3md");m.setColor("Color",i<2?ColorRGBA.Yellow:i<4?ColorRGBA.Red:ColorRGBA.Blue);g.setMaterial(m);node.attachChild(g);rootNode.attachChild(node);node.setLocalTranslation(.8f,radius+.002f,-.6f+i*.24f);
            var body=new com.jme3.bullet.control.RigidBodyControl(new com.jme3.bullet.collision.shapes.SphereCollisionShape(radius),i<2?.0209836f:.0405855f);node.addControl(body);body.setFriction(simConfig.field.friction());body.setRestitution(simConfig.field.restitution());body.setRollingFriction(.005f);physicsWorld.space().add(body);physicsWorld.registerPiece(node.getName(),type,node,body);
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
