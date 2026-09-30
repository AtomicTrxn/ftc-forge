package simrunner;

import com.jme3.app.SimpleApplication;
import com.jme3.app.state.ScreenshotAppState;
import com.jme3.bounding.BoundingBox;
import com.jme3.font.BitmapText;
import com.jme3.input.ChaseCamera;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.system.AppSettings;
import com.qualcomm.robotcore.hardware.HardwareMap;
import simcore.RobotUrdf;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;

/** Inspect exported CAD without inventing collisions, motor bindings, or physical joints. */
public final class RobotPreviewApp extends SimpleApplication {
    private final Path urdfPath, screenshotPath;
    private ScreenshotAppState screenshot;
    private int frames;

    private RobotPreviewApp(Path urdfPath, Path screenshotPath) {
        this.urdfPath = urdfPath;
        this.screenshotPath = screenshotPath;
    }

    public static void main(String[] args) {
        if (args.length < 1 || args.length > 2) {
            System.err.println("Usage: RobotPreviewApp <robot.urdf> [screenshot.png]");
            System.exit(2);
        }
        var app = new RobotPreviewApp(Path.of(args[0]), args.length == 2 ? Path.of(args[1]).toAbsolutePath() : null);
        AppSettings settings = new AppSettings(true);
        settings.setTitle("FTC Forge — CAD preview");
        settings.setResolution(1280, 800);
        app.setSettings(settings);
        app.setShowSettings(false);
        if (app.screenshotPath != null) app.setPauseOnLostFocus(false);
        app.start();
    }

    @Override public void simpleInitApp() {
        try {
            RobotUrdf robot = RobotUrdf.parse(urdfPath);
            ImportedRobotScene scene = new ImportedRobotScene(robot, urdfPath, new HardwareMap(), assetManager, 8);
            rootNode.attachChild(scene.root);
            rootNode.addLight(new AmbientLight(new ColorRGBA(.25f, .25f, .25f, 1)));
            rootNode.addLight(new DirectionalLight(new Vector3f(-1, -2, -1).normalizeLocal(), new ColorRGBA(.7f, .7f, .7f, 1)));
            rootNode.updateGeometricState();
            if (!(scene.root.getWorldBound() instanceof BoundingBox box))
                throw new IllegalArgumentException("URDF contains no visible geometry");
            float radius = Math.max(box.getXExtent(), Math.max(box.getYExtent(), box.getZExtent()));
            cam.setFrustumPerspective(45, (float) cam.getWidth() / cam.getHeight(), Math.max(.001f, radius / 100), Math.max(10, radius * 100));
            flyCam.setEnabled(false);
            ChaseCamera orbit = new ChaseCamera(cam, scene.root, inputManager);
            orbit.setLookAtOffset(box.getCenter());
            orbit.setDefaultDistance(radius * 3.4f);
            orbit.setMinDistance(radius * .25f);
            orbit.setMaxDistance(radius * 20);
            orbit.setDefaultHorizontalRotation(.75f);
            orbit.setDefaultVerticalRotation(.45f);
            orbit.setDragToRotate(true);
            viewPort.setBackgroundColor(new ColorRGBA(.16f, .18f, .21f, 1));
            setDisplayStatView(false);
            setDisplayFps(false);
            long visuals = robot.links.values().stream().mapToLong(link -> link.visuals().size()).sum();
            long collisions = robot.links.values().stream().mapToLong(link -> link.collisions().size()).sum();
            long moving = robot.joints.values().stream().filter(joint -> !joint.type().equals("fixed")).count();
            String status = String.format(Locale.ROOT,
                "CAD preview — drag to orbit, scroll to zoom\n%d links | %d visuals | %d movable joints | %d collision shapes\nCAD mass: %.3f kg — geometry preview only",
                robot.links.size(), visuals, moving, collisions, robot.totalMassKg());
            BitmapText text = new BitmapText(guiFont);
            text.setText(status);
            text.setLocalTranslation(15, cam.getHeight() - 15, 0);
            guiNode.attachChild(text);
            System.out.printf(Locale.ROOT, "[PREVIEW] %s%n[PREVIEW] Bounds xyz: %.4f x %.4f x %.4f m%n",
                status.replace('\n', ' '), box.getXExtent() * 2, box.getZExtent() * 2, box.getYExtent() * 2);
            if (screenshotPath != null) {
                Files.createDirectories(screenshotPath.getParent());
                String name = screenshotPath.getFileName().toString();
                if (!name.endsWith(".png")) throw new IllegalArgumentException("Screenshot path must end in .png");
                screenshot = new ScreenshotAppState(screenshotPath.getParent() + "/", name.substring(0, name.length() - 4));
                screenshot.setIsNumbered(false);
                stateManager.attach(screenshot);
            }
        } catch (Exception error) {
            throw new IllegalArgumentException("Could not preview " + urdfPath + ": " + error.getMessage(), error);
        }
    }

    @Override public void simpleUpdate(float dt) {
        if (screenshot != null) {
            if (++frames == 3) screenshot.takeScreenshot();
            if (frames == 10) stop();
        }
    }
}
