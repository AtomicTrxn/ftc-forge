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

/** Visual URDF link tree and chassis collision made from its rigid, fixed-link subtree. */
final class ImportedRobotScene {
    private static final Quaternion BASIS = new Quaternion().fromAngleAxis(-FastMath.HALF_PI, Vector3f.UNIT_X);
    private final RobotUrdf urdf;
    private final Path urdfPath;
    private final HardwareMap hardwareMap;
    private final AssetManager assets;
    private final int vhacdMaxHulls;
    final Node root = new Node("imported-robot");
    private final Map<String, Node> jointNodes = new LinkedHashMap<>();
    private final Map<String, Node> linkNodes = new LinkedHashMap<>();

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
            if (joint.type().equals("fixed")) continue;
            Node mount = jointNodes.get(joint.name());
            double value = urdf.jointPosition(joint.name(), hardwareMap);
            Vector3f axis = position(joint.axis()).normalizeLocal();
            if (joint.type().equals("prismatic")) {
                mount.setLocalTranslation(position(joint.origin().xyz()).add(axis.mult((float) value)));
            } else {
                mount.setLocalRotation(rotation(joint.origin().rpy())
                    .mult(new Quaternion().fromAngleAxis((float) value, axis)));
            }
        }
    }

    /** Aggregate CAD inertia about the chassis origin, including current mechanism positions. */
    Vector3f inertiaDiagonal() {
        root.updateGeometricState();
        double[] diagonal = new double[3];
        for (RobotUrdf.Link link : urdf.links.values()) {
            Node node = linkNodes.get(link.name());
            Vector3f comWorld = node.getWorldTranslation().add(
                node.getWorldRotation().mult(position(link.inertialOrigin().xyz())));
            Vector3f com = root.worldToLocal(comWorld, null);
            Quaternion q = root.getWorldRotation().inverse().mult(node.getWorldRotation())
                .mult(rotation(link.inertialOrigin().rpy()));
            com.jme3.math.Matrix3f r = q.toRotationMatrix();
            double[][] matrix = {
                {link.inertia().ixx(), link.inertia().ixz(), -link.inertia().ixy()},
                {link.inertia().ixz(), link.inertia().izz(), -link.inertia().iyz()},
                {-link.inertia().ixy(), -link.inertia().iyz(), link.inertia().iyy()}
            };
            double[][] rows = new double[3][3];
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) rows[i][j] = r.get(i, j);
            for (int a = 0; a < 3; a++) {
                for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
                    diagonal[a] += rows[a][i] * matrix[i][j] * rows[a][j];
            }
            diagonal[0] += link.massKg() * (com.y * com.y + com.z * com.z);
            diagonal[1] += link.massKg() * (com.x * com.x + com.z * com.z);
            diagonal[2] += link.massKg() * (com.x * com.x + com.y * com.y);
        }
        for (double moment : diagonal) {
            if (!Double.isFinite(moment) || moment <= 0)
                throw new IllegalArgumentException("Imported URDF has non-positive aggregate inertia");
        }
        return new Vector3f((float) diagonal[0], (float) diagonal[1], (float) diagonal[2]);
    }

    /** Moving joints remain visual/encoder tracked; fixed link geometry is welded to chassis. */
    CollisionShape chassisShape() throws Exception {
        CompoundCollisionShape result = new CompoundCollisionShape();
        addFixedCollision(urdf.rootLink, Vector3f.ZERO, new Quaternion(), result);
        if (result.countChildren() == 0) throw new IllegalArgumentException("URDF chassis has no fixed collision geometry");
        return result;
    }

    private void addFixedCollision(String linkName, Vector3f offset, Quaternion orientation,
                                   CompoundCollisionShape result) throws Exception {
        RobotUrdf.Link link = urdf.links.get(linkName);
        for (RobotUrdf.Collision c : link.collisions()) {
            Vector3f local = position(c.origin().xyz());
            Vector3f translated = offset.add(orientation.mult(local));
            Quaternion rotated = orientation.mult(rotation(c.origin().rpy()))
                .mult(shapeAxisRotation(c.geometry()));
            result.addChildShape(physicsShape(c.geometry()), translated, rotated.toRotationMatrix());
        }
        for (RobotUrdf.Joint joint : urdf.joints.values()) {
            if (!joint.parent().equals(linkName) || !joint.type().equals("fixed")) continue;
            Vector3f childOffset = offset.add(orientation.mult(position(joint.origin().xyz())));
            Quaternion childRotation = orientation.mult(rotation(joint.origin().rpy()));
            addFixedCollision(joint.child(), childOffset, childRotation, result);
        }
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
        Quaternion urdfRotation = new Quaternion().fromAngles((float) rpy[0], (float) rpy[1], (float) rpy[2]);
        return BASIS.mult(urdfRotation).mult(BASIS.inverse());
    }

    private static Quaternion shapeAxisRotation(RobotUrdf.Geometry geometry) {
        // jME Cylinder's long axis is Z; URDF's is Z, which maps to jME Y in our frame.
        return geometry.kind().equals("cylinder") ? BASIS : new Quaternion();
    }
}
