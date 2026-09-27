package simrunner;

import com.jme3.asset.AssetManager;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Cylinder;
import com.jme3.scene.shape.Sphere;
import com.qualcomm.robotcore.hardware.HardwareMap;
import simcore.RobotUrdf;
import vhacd4.Vhacd4;
import vhacd4.Vhacd4Hull;
import vhacd4.Vhacd4Parameters;

import java.nio.FloatBuffer;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.List;
import java.util.ArrayList;
import com.jme3.math.Transform;
import com.jme3.bullet.collision.shapes.EmptyShape;

/** URDF geometry grouped into fixed subtrees for articulated rigid-body construction. */
final class ImportedRobotScene {
    private static final Quaternion BASIS = new Quaternion().fromAngleAxis(-FastMath.HALF_PI, Vector3f.UNIT_X);
    final RobotUrdf urdf;
    private final Path urdfPath;
    final HardwareMap hardwareMap;
    private final AssetManager assets;
    private final int vhacdMaxHulls;
    final Node root = new Node("imported-robot");
    private final Map<String, Node> jointNodes = new LinkedHashMap<>();
    final Map<String, Node> linkNodes = new LinkedHashMap<>();

    ImportedRobotScene(RobotUrdf urdf, Path urdfPath, HardwareMap hardwareMap, AssetManager assets,
                       int vhacdMaxHulls) throws Exception {
        this.urdf = urdf;
        this.urdfPath = urdfPath;
        this.hardwareMap = hardwareMap;
        this.assets = assets;
        this.vhacdMaxHulls = vhacdMaxHulls;
        addLink(urdf.rootLink, root);
    }

    private void addLink(String linkName, Node parent) throws Exception {
        RobotUrdf.Link link = urdf.links.get(linkName);
        Node linkNode = new Node(linkName);
        parent.attachChild(linkNode);
        linkNodes.put(linkName, linkNode);
        for (RobotUrdf.Collision c : link.collisions()) {
            Geometry geometry = new Geometry(linkName + "-collision", visualMesh(c.geometry()));
            Material material = new Material(assets, "Common/MatDefs/Misc/Unshaded.j3md");
            material.setColor("Color", linkName.equals(urdf.rootLink) ? ColorRGBA.Blue : ColorRGBA.Gray);
            geometry.setMaterial(material);
            geometry.setLocalTranslation(position(c.origin().xyz()));
            geometry.setLocalRotation(rotation(c.origin().rpy()).mult(shapeAxisRotation(c.geometry())));
            linkNode.attachChild(geometry);
        }
        for (RobotUrdf.Joint joint : urdf.joints.values()) {
            if (!joint.parent().equals(linkName)) continue;
            Node mount = new Node(joint.name());
            mount.setLocalTranslation(position(joint.origin().xyz()));
            mount.setLocalRotation(rotation(joint.origin().rpy()));
            linkNode.attachChild(mount);
            jointNodes.put(joint.name(), mount);
            addLink(joint.child(), mount);
        }
    }

    void update() {
        for (RobotUrdf.Joint joint : urdf.joints.values()) {
            if (joint.type().equals("fixed") || !wheelLinks.contains(joint.child())) continue;
            Node mount = jointNodes.get(joint.name());
            double value = urdf.jointPosition(joint.name(), hardwareMap);
            Vector3f axis = position(joint.axis()).normalizeLocal();
            if (joint.type().equals("prismatic")) {
                mount.setLocalTranslation(position(joint.origin().xyz()).add(rotation(joint.origin().rpy()).mult(axis.mult((float) value))));
            } else {
                mount.setLocalRotation(rotation(joint.origin().rpy())
                    .mult(new Quaternion().fromAngleAxis((float) value, axis)));
            }
        }
    }

    record Part(String name, Node visual, Transform origin, Vector3f com, double mass,
                Vector3f inertia, CollisionShape shape) { }
    final Map<String, String> owners = new LinkedHashMap<>();
    final java.util.Set<String> wheelLinks = new java.util.HashSet<>();

    boolean isDriveWheel(RobotUrdf.Joint joint) {
        return urdf.transmissions.values().stream().filter(t -> t.joint().equals(joint.name()))
            .flatMap(t -> t.actuators().stream()).anyMatch(a -> List.of("left_front_drive",
                "right_front_drive", "left_back_drive", "right_back_drive").contains(a.name()));
    }

    private void assignParts(String link, String owner, boolean wheelBranch) {
        owners.put(link, owner);
        if (wheelBranch) wheelLinks.add(link);
        for (RobotUrdf.Joint joint : urdf.joints.values()) {
            if (!joint.parent().equals(link)) continue;
            boolean wheel = wheelBranch || isDriveWheel(joint);
            boolean separate = !joint.type().equals("fixed") && !wheel;
            assignParts(joint.child(), separate ? joint.child() : owner, wheel);
        }
    }

    /** Weld fixed subtrees and keep wheel mass as ballast without wheel-ground traction. */
    List<Part> parts() throws Exception {
        root.updateGeometricState();
        owners.clear();
        wheelLinks.clear();
        assignParts(urdf.rootLink, urdf.rootLink, false);
        List<Part> result = new ArrayList<>();
        for (String name : new java.util.LinkedHashSet<>(owners.values())) {
            Node originNode = linkNodes.get(name);
            Transform origin = originNode.getWorldTransform().clone();
            Quaternion inverse = origin.getRotation().inverse();
            double mass = 0;
            Vector3f weightedCom = new Vector3f();
            Map<String, Vector3f> centers = new LinkedHashMap<>();
            for (RobotUrdf.Link link : urdf.links.values()) {
                if (!owners.get(link.name()).equals(name)) continue;
                Node node = linkNodes.get(link.name());
                Vector3f center = inverse.mult(node.localToWorld(position(link.inertialOrigin().xyz()), null)
                    .subtract(origin.getTranslation()));
                centers.put(link.name(), center);
                weightedCom.addLocal(center.mult((float) link.massKg()));
                mass += link.massKg();
            }
            if (mass <= 0) throw new IllegalArgumentException("Dynamic URDF subtree has no positive mass: " + name);
            Vector3f com = weightedCom.divide((float) mass);
            double[] diagonal = new double[3];
            CompoundCollisionShape compound = new CompoundCollisionShape();
            for (RobotUrdf.Link link : urdf.links.values()) {
                if (!owners.get(link.name()).equals(name)) continue;
                Node node = linkNodes.get(link.name());
                Quaternion relative = inverse.mult(node.getWorldRotation());
                Quaternion inertialRotation = relative.mult(rotation(link.inertialOrigin().rpy()));
                var r = inertialRotation.toRotationMatrix();
                double[][] tensor = {{link.inertia().ixx(), link.inertia().ixz(), -link.inertia().ixy()},
                    {link.inertia().ixz(), link.inertia().izz(), -link.inertia().iyz()},
                    {-link.inertia().ixy(), -link.inertia().iyz(), link.inertia().iyy()}};
                for (int a = 0; a < 3; a++) for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
                    diagonal[a] += r.get(a, i) * tensor[i][j] * r.get(a, j);
                Vector3f offset = centers.get(link.name()).subtract(com);
                diagonal[0] += link.massKg() * (offset.y * offset.y + offset.z * offset.z);
                diagonal[1] += link.massKg() * (offset.x * offset.x + offset.z * offset.z);
                diagonal[2] += link.massKg() * (offset.x * offset.x + offset.y * offset.y);
                if (wheelLinks.contains(link.name())) continue;
                for (RobotUrdf.Collision collision : link.collisions()) {
                    Vector3f translation = inverse.mult(node.localToWorld(position(collision.origin().xyz()), null)
                        .subtract(origin.getTranslation())).subtract(com);
                    Quaternion orientation = relative.mult(rotation(collision.origin().rpy()))
                        .mult(shapeAxisRotation(collision.geometry()));
                    compound.addChildShape(physicsShape(collision.geometry()), translation, orientation.toRotationMatrix());
                }
            }
            for (double moment : diagonal) if (!Double.isFinite(moment) || moment <= 0)
                throw new IllegalArgumentException("Dynamic URDF subtree has invalid inertia: " + name);
            CollisionShape shape = compound.countChildren() == 0 ? new EmptyShape(false) : compound;
            result.add(new Part(name, originNode, origin, com, mass,
                new Vector3f((float) diagonal[0], (float) diagonal[1], (float) diagonal[2]), shape));
        }
        return result;
    }

    private Mesh visualMesh(RobotUrdf.Geometry g) throws Exception {
        return switch (g.kind()) {
            case "box" -> new Box((float) g.dimensions()[0] / 2, (float) g.dimensions()[2] / 2,
                (float) g.dimensions()[1] / 2);
            case "cylinder" -> new Cylinder(12, 24, (float) g.dimensions()[0], (float) g.dimensions()[1], true);
            case "sphere" -> new Sphere(12, 24, (float) g.dimensions()[0]);
            case "mesh" -> StlMeshLoader.load(resolveMesh(g.meshFile()), g.meshScale());
            default -> throw new IllegalArgumentException("Unsupported geometry " + g.kind());
        };
    }

    private CollisionShape physicsShape(RobotUrdf.Geometry g) throws Exception {
        return switch (g.kind()) {
            case "box" -> new BoxCollisionShape(new Vector3f((float) g.dimensions()[0] / 2,
                (float) g.dimensions()[2] / 2, (float) g.dimensions()[1] / 2));
            case "sphere" -> new SphereCollisionShape((float) g.dimensions()[0]);
            case "cylinder", "mesh" -> decompose(visualMesh(g));
            default -> throw new IllegalArgumentException("Unsupported geometry " + g.kind());
        };
    }

    private CollisionShape decompose(Mesh mesh) {
        FloatBuffer positions = mesh.getFloatBuffer(com.jme3.scene.VertexBuffer.Type.Position).duplicate();
        positions.rewind();
        float[] vertices = new float[positions.remaining()];
        positions.get(vertices);
        var indices = mesh.getIndexBuffer();
        int[] triangles = new int[indices.size()];
        for (int i = 0; i < triangles.length; i++) triangles[i] = indices.get(i);
        Vhacd4Parameters params = new Vhacd4Parameters();
        params.setMaxHulls(vhacdMaxHulls);
        var hulls = Vhacd4.compute(vertices, triangles, params);
        if (hulls.isEmpty()) throw new IllegalArgumentException("V-HACD produced no hulls for imported mesh");
        CompoundCollisionShape result = new CompoundCollisionShape();
        for (Vhacd4Hull hull : hulls) result.addChildShape(new HullCollisionShape(hull));
        System.out.println("[IMPORT] V-HACD produced " + hulls.size() + " hulls for imported geometry");
        return result;
    }

    private Path resolveMesh(String file) {
        String path = file;
        if (path.startsWith("package://")) {
            path = path.substring("package://".length());
        }
        Path resolved = urdfPath.toAbsolutePath().getParent().resolve(path).normalize();
        if (!java.nio.file.Files.exists(resolved) && file.startsWith("package://") && path.contains("/")) {
            resolved = urdfPath.toAbsolutePath().getParent().resolve(path.substring(path.indexOf('/') + 1)).normalize();
        }
        if (!resolved.toString().toLowerCase().endsWith(".stl"))
            throw new IllegalArgumentException("Imported mesh must be STL: " + file);
        return resolved;
    }

    static Vector3f position(double[] xyz) {
        return new Vector3f((float) xyz[0], (float) xyz[2], (float) -xyz[1]);
    }
    static Quaternion rotation(double[] rpy) {
        Quaternion urdfRotation = new Quaternion().fromAngleAxis((float) rpy[2], Vector3f.UNIT_Z)
            .mult(new Quaternion().fromAngleAxis((float) rpy[1], Vector3f.UNIT_Y))
            .mult(new Quaternion().fromAngleAxis((float) rpy[0], Vector3f.UNIT_X));
        return BASIS.mult(urdfRotation).mult(BASIS.inverse());
    }

    private static Quaternion shapeAxisRotation(RobotUrdf.Geometry geometry) {
        // jME Cylinder's long axis is Z; URDF's is Z, which maps to jME Y in our frame.
        return geometry.kind().equals("cylinder") ? BASIS : new Quaternion();
    }
}
