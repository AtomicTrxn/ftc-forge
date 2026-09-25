package simcore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import javax.xml.parsers.DocumentBuilderFactory;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.HashSet;

/** URDF geometry and transmission bindings paired with the FTC robot-configuration XML. */
public final class RobotUrdf {
    public record Pose(double[] xyz, double[] rpy) {}
    public record Geometry(String kind, double[] dimensions, String meshFile, double[] meshScale) {}
    public record Collision(Pose origin, Geometry geometry) {}
    public record Inertia(double ixx, double iyy, double izz, double ixy, double ixz, double iyz) {
        Inertia scaled(double factor) {
            return new Inertia(ixx * factor, iyy * factor, izz * factor,
                ixy * factor, ixz * factor, iyz * factor);
        }
    }
    public record Link(String name, double massKg, Pose inertialOrigin, Inertia inertia,
                       List<Collision> collisions) {
        Link scaled(double factor) {
            return new Link(name, massKg * factor, inertialOrigin, inertia.scaled(factor), collisions);
        }
    }
    public record Joint(String name, String type, String parent, String child, Pose origin,
                        double[] axis, Double lower, Double upper) {}
    public record Actuator(String name, double mechanicalReduction) {}
    public record Transmission(String name, String joint, List<Actuator> actuators) {}

    public final String name;
    public final Map<String, Link> links;
    public final Map<String, Joint> joints;
    public final Map<String, Transmission> transmissions;
    public final String rootLink;

    private RobotUrdf(String name, Map<String, Link> links, Map<String, Joint> joints,
                      Map<String, Transmission> transmissions, String rootLink) {
        this.name = name;
        this.links = Map.copyOf(links);
        this.joints = Map.copyOf(joints);
        this.transmissions = Map.copyOf(transmissions);
        this.rootLink = rootLink;
    }

    public double totalMassKg() {
        return links.values().stream().mapToDouble(Link::massKg).sum();
    }

    /** Scale measured mass and all CAD inertia tensor entries by the same factor. */
    public RobotUrdf withTotalMassKg(double measuredKg) {
        if (!Double.isFinite(measuredKg) || measuredKg <= 0 || totalMassKg() <= 0)
            throw new IllegalArgumentException("total_mass_kg and CAD mass must be positive and finite");
        double factor = measuredKg / totalMassKg();
        Map<String, Link> scaled = new LinkedHashMap<>();
        links.forEach((key, value) -> scaled.put(key, value.scaled(factor)));
        return new RobotUrdf(name, scaled, joints, transmissions, rootLink);
    }

    public static RobotUrdf parse(Path path) throws Exception {
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        factory.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
        factory.setFeature("http://xml.org/sax/features/external-general-entities", false);
        factory.setFeature("http://xml.org/sax/features/external-parameter-entities", false);
        factory.setXIncludeAware(false);
        factory.setExpandEntityReferences(false);
        Element robot = factory.newDocumentBuilder().parse(path.toFile()).getDocumentElement();
        if (!"robot".equals(robot.getTagName())) throw new IllegalArgumentException("URDF root must be <robot>");
        Map<String, Link> links = new LinkedHashMap<>();
        Map<String, Joint> joints = new LinkedHashMap<>();
        Map<String, Transmission> transmissions = new LinkedHashMap<>();
        for (Element el : children(robot, "link")) {
            String name = required(el, "name");
            Element inertial = child(el, "inertial");
            if (inertial == null) throw new IllegalArgumentException("Link " + name + " has no <inertial>");
            double mass = number(requiredChild(inertial, "mass"), "value");
            if (mass < 0) throw new IllegalArgumentException("Negative mass on link " + name);
            Element tensor = requiredChild(inertial, "inertia");
            Inertia inertia = new Inertia(number(tensor, "ixx"), number(tensor, "iyy"), number(tensor, "izz"),
                number(tensor, "ixy"), number(tensor, "ixz"), number(tensor, "iyz"));
            List<Collision> collisions = new ArrayList<>();
            for (Element c : children(el, "collision")) {
                Element shape = requiredChild(c, "geometry");
                Geometry geometry;
                if (child(shape, "box") != null) {
                    geometry = new Geometry("box", vector(requiredChild(shape, "box").getAttribute("size"), 3), null, null);
                } else if (child(shape, "cylinder") != null) {
                    Element cylinder = child(shape, "cylinder");
                    geometry = new Geometry("cylinder", new double[]{number(cylinder, "radius"), number(cylinder, "length")}, null, null);
                } else if (child(shape, "sphere") != null) {
                    geometry = new Geometry("sphere", new double[]{number(child(shape, "sphere"), "radius")}, null, null);
                } else if (child(shape, "mesh") != null) {
                    Element mesh = child(shape, "mesh");
                    geometry = new Geometry("mesh", null, required(mesh, "filename"),
                        mesh.hasAttribute("scale") ? vector(mesh.getAttribute("scale"), 3) : new double[]{1, 1, 1});
                } else throw new IllegalArgumentException("Unsupported collision geometry on link " + name);
                if (geometry.dimensions() != null) {
                    for (double size : geometry.dimensions())
                        if (size <= 0) throw new IllegalArgumentException("Non-positive collision size on link " + name);
                }
                if (geometry.meshScale() != null) {
                    for (double scale : geometry.meshScale())
                        if (scale == 0) throw new IllegalArgumentException("Zero mesh scale on link " + name);
                }
                collisions.add(new Collision(pose(c), geometry));
            }
            if (links.putIfAbsent(name, new Link(name, mass, pose(inertial), inertia, List.copyOf(collisions))) != null)
                throw new IllegalArgumentException("Duplicate URDF link " + name);
        }
        if (links.isEmpty()) throw new IllegalArgumentException("URDF has no links");
        Set<String> children = new HashSet<>();
        for (Element el : children(robot, "joint")) {
            String name = required(el, "name"), type = required(el, "type");
            if (!Set.of("fixed", "continuous", "revolute", "prismatic").contains(type))
                throw new IllegalArgumentException("Unsupported joint type " + type + " on " + name);
            String parent = required(requiredChild(el, "parent"), "link");
            String child = required(requiredChild(el, "child"), "link");
            if (!links.containsKey(parent) || !links.containsKey(child) || parent.equals(child))
                throw new IllegalArgumentException("Invalid links on joint " + name);
            if (!children.add(child)) throw new IllegalArgumentException("Link has multiple parent joints: " + child);
            Element limit = child(el, "limit");
            Double lower = null, upper = null;
            if (type.equals("revolute") || type.equals("prismatic")) {
                if (limit == null) throw new IllegalArgumentException("Missing limit on joint " + name);
                lower = number(limit, "lower"); upper = number(limit, "upper");
                if (lower > upper) throw new IllegalArgumentException("Inverted limits on joint " + name);
            }
            Element axis = child(el, "axis");
            double[] xyz = axis == null ? new double[]{1, 0, 0} : vector(required(axis, "xyz"), 3);
            if (!type.equals("fixed") && xyz[0] == 0 && xyz[1] == 0 && xyz[2] == 0)
                throw new IllegalArgumentException("Zero axis on joint " + name);
            Joint joint = new Joint(name, type, parent, child, pose(el), xyz, lower, upper);
            if (joints.putIfAbsent(name, joint) != null) throw new IllegalArgumentException("Duplicate joint " + name);
        }
        List<String> roots = links.keySet().stream().filter(k -> !children.contains(k)).toList();
        if (roots.size() != 1 || joints.size() != links.size() - 1)
            throw new IllegalArgumentException("URDF links must form one rooted tree");
        String rootLink = roots.get(0);
        Set<String> reached = new HashSet<>();
        visit(rootLink, joints, reached);
        if (reached.size() != links.size()) throw new IllegalArgumentException("URDF joint tree has a cycle or disconnected link");
        for (Element el : children(robot, "transmission")) {
            String name = required(el, "name");
            String joint = required(requiredChild(el, "joint"), "name");
            if (!joints.containsKey(joint) || joints.get(joint).type().equals("fixed"))
                throw new IllegalArgumentException("Transmission " + name + " references missing/fixed joint " + joint);
            List<Actuator> actuators = new ArrayList<>();
            for (Element a : children(el, "actuator")) {
                Element reduction = child(a, "mechanicalReduction");
                double value = reduction == null ? 1 : Double.parseDouble(reduction.getTextContent().trim());
                if (!Double.isFinite(value) || value == 0) throw new IllegalArgumentException("Invalid reduction in " + name);
                actuators.add(new Actuator(required(a, "name"), value));
            }
            if (actuators.isEmpty()) throw new IllegalArgumentException("Transmission " + name + " has no actuators");
            if (transmissions.putIfAbsent(name, new Transmission(name, joint, List.copyOf(actuators))) != null)
                throw new IllegalArgumentException("Duplicate transmission " + name);
        }
        return new RobotUrdf(required(robot, "name"), links, joints, transmissions, rootLink);
    }

    /** Resolve every actuator against the actual map built from the paired FTC XML. */
    public void validateHardwareMap(HardwareMap map) {
        for (Transmission tx : transmissions.values()) {
            for (Actuator actuator : tx.actuators()) {
                Object device = map.tryGet(com.qualcomm.robotcore.hardware.HardwareDevice.class, actuator.name());
                if (!(device instanceof DcMotorEx) && !(device instanceof Servo))
                    throw new IllegalArgumentException("Transmission " + tx.name() + " actuator \"" + actuator.name()
                        + "\" does not resolve to a DcMotorEx or Servo in the paired robot-configuration XML");
            }
        }
    }

    /** Joint position in radians or meters; motor ticks are output-shaft radians before reduction. */
    public double jointPosition(String jointName, HardwareMap map) {
        Joint joint = joints.get(jointName);
        if (joint == null) throw new IllegalArgumentException("Unknown joint " + jointName);
        double total = 0; int count = 0;
        for (Transmission tx : transmissions.values()) {
            if (!tx.joint().equals(jointName)) continue;
            for (Actuator actuator : tx.actuators()) {
                DcMotorEx motor = map.tryGet(DcMotorEx.class, actuator.name());
                if (motor instanceof SimDcMotorEx simMotor) {
                    double turns = (double) motor.getCurrentPosition() / simMotor.getSpec().encoderCountsPerRev;
                    total += turns * 2 * Math.PI / actuator.mechanicalReduction();
                } else {
                    Servo servo = map.get(Servo.class, actuator.name());
                    double travel = joint.lower() != null ? joint.upper() - joint.lower() : 2 * Math.PI;
                    total += (joint.lower() == null ? 0 : joint.lower()) + servo.getPosition() * travel / actuator.mechanicalReduction();
                }
                count++;
            }
        }
        if (count == 0) return 0;
        double value = total / count;
        if (joint.lower() != null) value = Math.max(joint.lower(), Math.min(joint.upper(), value));
        return value;
    }

    private static void visit(String link, Map<String, Joint> joints, Set<String> reached) {
        if (!reached.add(link)) throw new IllegalArgumentException("URDF joint tree has a cycle at " + link);
        joints.values().stream().filter(j -> j.parent().equals(link)).forEach(j -> visit(j.child(), joints, reached));
    }
    private static List<Element> children(Element parent, String tag) {
        List<Element> result = new ArrayList<>();
        NodeList nodes = parent.getChildNodes();
        for (int i = 0; i < nodes.getLength(); i++) {
            Node n = nodes.item(i);
            if (n instanceof Element e && e.getTagName().equals(tag)) result.add(e);
        }
        return result;
    }
    private static Element child(Element parent, String tag) {
        List<Element> matches = children(parent, tag);
        return matches.isEmpty() ? null : matches.get(0);
    }
    private static Element requiredChild(Element parent, String tag) {
        Element e = child(parent, tag);
        if (e == null) throw new IllegalArgumentException("Missing <" + tag + "> in <" + parent.getTagName() + ">");
        return e;
    }
    private static String required(Element el, String attr) {
        String value = el.getAttribute(attr);
        if (value.isBlank()) throw new IllegalArgumentException("Missing " + attr + " on <" + el.getTagName() + ">");
        return value;
    }
    private static double number(Element el, String attr) {
        double value = Double.parseDouble(required(el, attr));
        if (!Double.isFinite(value)) throw new IllegalArgumentException("Non-finite " + attr + " on <" + el.getTagName() + ">");
        return value;
    }
    private static double[] vector(String value, int size) {
        String[] parts = value.trim().split("\\s+");
        if (parts.length != size) throw new IllegalArgumentException("Expected " + size + " values: " + value);
        double[] values = new double[size];
        for (int i = 0; i < size; i++) {
            values[i] = Double.parseDouble(parts[i]);
            if (!Double.isFinite(values[i])) throw new IllegalArgumentException("Non-finite vector value: " + value);
        }
        return values;
    }
    private static Pose pose(Element parent) {
        Element origin = child(parent, "origin");
        return origin == null ? new Pose(new double[3], new double[3]) : new Pose(
            origin.hasAttribute("xyz") ? vector(origin.getAttribute("xyz"), 3) : new double[3],
            origin.hasAttribute("rpy") ? vector(origin.getAttribute("rpy"), 3) : new double[3]);
    }
}
