package simrunner;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.*;
import com.jme3.scene.Node;
import com.qualcomm.robotcore.hardware.*;
import physics.MecanumKinematics;
import simcore.*;
import java.nio.file.Path;
import java.util.*;

/** No application, renderer or window. Uses the same native scene classes as SimulatorApp. */
final class HeadlessScene implements AutoCloseable {
    final PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
    final Node root = new Node("headless-scene");
    final PhysicsWorld world;
    final HardwareMap hardware;
    final SimConfig config;
    ImportedRobotScene imported;
    ArticulatedRobot robot;
    SceneSensors sensors;
    FieldBehavior behavior;
    final String[] motorNames;
    private MecanumKinematics kinematics;
    private List<Double> radii;
    double seconds;
    Map<String,Object> description;

    HeadlessScene(Path project, SimConfig config) throws Exception {
        this.config = config;
        var assets = new DesktopAssetManager(true);
        world = new PhysicsWorld(assets, root, space);
        motorNames = config.drive == null ? new String[]{"left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive"}
            : config.drive.motorNames().toArray(String[]::new);
        try {
            hardware = HardwareMapBuilder.build(RobotConfigXml.parse(project.resolve(config.robotConfig).toFile()),
                PresetRobotConfig.load(project.resolve(config.presetMotors)));
            if (config.calibration != null) {
                var calibration = CalibrationProfile.load(project.resolve(config.calibration));
                calibration.applyHardware(hardware); calibration.applyDrive(world);
            }
            for (var imu : hardware.getAll(SimIMU.class)) imu.setLatencyMs(config.imuLatencyMs);
            for (String name : motorNames) hardware.get(SimDcMotorEx.class, name);
            if (config.intake != null) hardware.get(SimDcMotorEx.class, config.intake.motor());
            var environment = new HashSet<Long>();
            Vector3f half = new Vector3f(1.8288f, 0, 1.8288f);
            float floor = 0;
            ImportedFieldScene legacyField = null;
            var field = config.field.validated();
            if (field.source().equals("imported")) {
                Path file = project.resolve(field.packagePath());
                if (file.getFileName().toString().equals("profile.json")) {
                    var profile = new ModelProfile(file, true);
                    if (!profile.kind.equals("field")) throw new IllegalArgumentException("Select a field profile");
                    if (config.fieldBehavior.isEmpty() && profile.runtime.containsKey("field_behavior"))
                        config.fieldBehavior = FieldPackage.map(profile.runtime.get("field_behavior"));
                    floor = (float)FieldPackage.num(profile.parameters, "floor_top_m");
                    if (profile.legacyField()) legacyField = new ImportedFieldScene(new FieldPackage(profile.artifact("field")), field, world, root, assets);
                    else {
                        var built = new ModelFieldScene(profile, field.hasPieces(), world, root, assets, config.scenePieces);
                        half = built.halfExtents;
                        for (var body : built.bodies) environment.add(body.nativeId());
                    }
                } else legacyField = new ImportedFieldScene(new FieldPackage(file), field, world, root, assets);
                if (legacyField != null) {
                    half = legacyField.field.halfExtents;
                    if (legacyField.field.modelParameters.containsKey("floor_top_m")) floor = (float)FieldPackage.num(legacyField.field.modelParameters, "floor_top_m");
                    for (var body : legacyField.fixed) environment.add(body.nativeId());
                    for (var hive : legacyField.hives) environment.add(hive.body().nativeId());
                }
                if (config.robotStart == null) config.robotStart = new Vector3f(-1.2f, (float)config.startHeightM, 1.2f);
            } else {
                world.buildFieldBoundary();
                if (field.hasPieces() && !field.usesTorus()) PracticePieces.build(world, root, assets, false,
                    config.flexibleIntake != null, config.scenePieces, field.friction(), field.restitution());
            }
            Vector3f start = config.robotStart == null ? new Vector3f(0, (float)config.startHeightM, 0) : config.robotStart;
            RobotUrdf urdf = null;
            if (config.urdf != null) {
                Path file = project.resolve(config.urdf);
                urdf = RobotUrdf.parse(file);
                if (config.totalMassKg != null) urdf = urdf.withTotalMassKg(config.totalMassKg);
                urdf.validateHardwareMap(hardware);
                imported = new ImportedRobotScene(urdf, file, hardware, assets, config.vhacdMaxHulls, Set.of(motorNames), field.fullDetail() ? 0 : .0005f);
                if (config.robotProfile != null) config.robotProfile.configure(imported);
                imported.tireContacts = config.tires != null;
                imported.driveContacts = config.driveContacts;
                imported.rotatingWheels = config.rotatingWheels;
                imported.flexibleIntake = config.flexibleIntake;
                imported.collisionOmissions = config.collisionOmissions;
                CollisionAudit.inspect(imported).requireUsable();
                SceneChecks.robotStart(imported, start, config.robotYawRad, half, floor, legacyField != null);
                if (config.rotatingWheels != null) world.driveControllerEnabled = false;
                robot = new ArticulatedRobot(imported, world, root, start, config.servoPhysics,
                    new Quaternion().fromAngleAxis(config.robotYawRad, Vector3f.UNIT_Y));
                if (config.tires != null) world.installTires(new TireDrive(world, hardware, imported, config.drive, config.tires));
                if (config.flexibleIntake != null) {
                    world.installFlexibleIntake(new FlexibleIntake(world, imported, robot, config.flexibleIntake));
                    if (config.torusRetention != null && field.usesTorus())
                        world.flexibleIntake().installRetention(robot, hardware, config.intake, config.torusRetention);
                }
            } else {
                SceneChecks.robotStart(null, start, config.robotYawRad, half, floor, legacyField != null);
                var node = new Node("robot"); root.attachChild(node);
                world.buildChassis(node, config.totalMassKg == null ? 8.5 : config.totalMassKg, start);
                world.chassisBody().setPhysicsRotation(new Quaternion().fromAngleAxis(config.robotYawRad, Vector3f.UNIT_Y));
            }
            if (config.drive == null) {
                var geometry = DriveGeometry.resolve(urdf, config.driveGeometry);
                radii = geometry.wheelRadii();
                kinematics = new MecanumKinematics(geometry.trackWidthM(), geometry.wheelbaseM(), 2);
            }
            if (field.usesTorus()) {
                var layout = config.scenePieces;
                if (layout.isEmpty() && config.gamePieceStart != null) {
                    var p = config.gamePieceStart;
                    layout = Map.of("practice-torus", Map.of("enabled", true, "source_id", "practice-torus",
                        "xyz_m", List.of((double)p.x, (double)-p.z, (double)p.y),
                        "rpy_rad", List.of(config.flexibleIntake == null ? 0. : Math.PI / 2, 0., 0.)));
                }
                PracticePieces.build(world, root, assets, true, config.flexibleIntake != null, layout, field.friction(), field.restitution());
            }
            if (field.hasPieces() && legacyField != null) {
                SceneProfile.legacyPieces(config.scenePieces, legacyField, world);
                SceneProfile.placePieces(config.scenePieces, world);
            }
            description = Map.of("field_half_extents_m", List.of((double)half.x, (double)half.z),
                "floor_top_m", floor, "field_mode", field.mode(), "piece_count", world.gamePieces().size(),
                "native_bodies", space.countRigidBodies(), "robot_mass_kg", urdf == null ? (config.totalMassKg == null ? 8.5 : config.totalMassKg) : urdf.totalMassKg());
            root.updateGeometricState();
            SceneChecks.contacts(world, environment, half, floor);
            space.setAccuracy(config.tires != null || config.rotatingWheels != null || config.flexibleIntake != null ? 1f / 480 : 1f / 120);
            sensors = new SceneSensors(world, robot, hardware, config.sensors, config.fieldBehavior);
            if (!config.fieldBehavior.isEmpty()) behavior = new FieldBehavior(world, config.fieldBehavior, field.hasPieces());
        } catch (Exception | Error e) { close(); throw e; }
    }

    void tick() { tick(true); }
    void tick(boolean publishOdometry) {
        float dt = space.getAccuracy();
        seconds += dt;
        HardwareMapBuilder.tickMotors(hardware, dt, Math.round(seconds * 1000));
        MecanumKinematics.ChassisVelocity velocity;
        if (config.drive != null) velocity = config.drive.velocity(hardware);
        else {
            double[] speeds = new double[4];
            for (int i = 0; i < 4; i++) {
                var motor = hardware.get(SimDcMotorEx.class, motorNames[i]);
                speeds[i] = motor.getOmegaRadS() * (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1) * radii.get(i);
            }
            velocity = kinematics.forwardFromWheelSpeeds(speeds[0], speeds[1], speeds[2], speeds[3]);
        }
        world.driveChassis(velocity, dt);
        space.update(dt, 0); // exactly one native step, never drop accumulated steps
        for (var body : space.getRigidBodyList()) if (!Vector3f.isValidVector(body.getPhysicsLocation())
                || body.isDynamic() && (!Vector3f.isValidVector(body.getLinearVelocity()) || !Vector3f.isValidVector(body.getAngularVelocity())))
            throw new IllegalStateException("Nonfinite native body state");
        publishPose(publishOdometry);
        var claw = hardware.tryGet(Servo.class, "claw");
        boolean active = claw != null && claw.getPosition() > .5;
        var point = world.getChassisPosition().add(world.getChassisRotation().mult(Vector3f.UNIT_X).mult(.35f));
        if (config.intake != null) { active = config.intake.active(hardware); point = world.robotPointWorld(config.intake.point()); }
        world.updateIntake(active, point, config.intake == null ? .15f : config.intake.captureRadiusM());
    }

    void publishPose() { publishPose(true); }
    private void publishPose(boolean odometry) {
        var pose = world.getChassisRotation(); var omega = world.getChassisAngularVelocity();
        for (var imu : hardware.getAll(SimIMU.class)) RobotOrientation.update(imu, pose, omega, Math.round(seconds * 1000));
        var heading = pose.mult(Vector3f.UNIT_X); var at = world.getChassisPosition(); var v = world.chassisBody().getLinearVelocity();
        if (odometry) for (var sink : hardware.getAll(PoseSink.class)) sink.onChassisPose(at.x, -at.z,
            Math.atan2(-heading.z, heading.x), v.x, -v.z, omega.y);
    }

    Map<String,Object> sample(double runSeconds) {
        var at = world.getChassisPosition(); var heading = world.getChassisRotation().mult(Vector3f.UNIT_X);
        var motors = new LinkedHashMap<String,Object>();
        for (var motor : hardware.getAll(SimDcMotorEx.class)) motors.put(motor.getDeviceName(),
            Map.of("ticks", motor.getCurrentPosition(), "power", motor.getPower(), "shaft_rad", motor.getPhysicalShaftRadians()));
        return Map.of("time_s", runSeconds, "native_time_s", seconds,
            "xyz_m", List.of((double)at.x, (double)-at.z, (double)at.y), "yaw_rad", Math.atan2(-heading.z, heading.x),
            "motors", motors, "joints", robot == null ? Map.of() : robot.jointPositions(), "piece_held", world.isPieceHeld());
    }

    Map<String,Object> diagnostics() {
        return PhysicsDiagnostics.data(new PhysicsDiagnostics(world, imported, robot, hardware, Set.of(motorNames)).capture());
    }
    public void close() {
        if (behavior != null) behavior.close();
        if (sensors != null) sensors.close();
        space.destroy();
    }
}
