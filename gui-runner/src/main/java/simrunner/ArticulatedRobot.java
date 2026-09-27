package simrunner;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.SliderJoint;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.qualcomm.robotcore.hardware.Servo;
import simcore.RobotUrdf;
import simcore.SimDcMotorEx;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Finite-effort, environment-collidable URDF mechanisms with physical encoder feedback. */
final class ArticulatedRobot implements PhysicsTickListener {
    private record Body(ImportedRobotScene.Part part, RigidBodyControl control, Node node) { }
    private final ImportedRobotScene scene;
    private final Map<String, Body> bodies = new LinkedHashMap<>();
    private final List<Axis> axes = new ArrayList<>();
    final Node chassisNode;

    ArticulatedRobot(ImportedRobotScene scene, PhysicsWorld world, Node field, Vector3f start) throws Exception {
        this.scene = scene;
        List<ImportedRobotScene.Part> parts = scene.parts();
        // Snapshot joint origins before detaching/reparenting the visual tree.
        Map<String, Vector3f> jointPivots = new LinkedHashMap<>();
        Map<String, Quaternion> jointRotations = new LinkedHashMap<>();
        for (RobotUrdf.Joint joint : scene.urdf.joints.values()) {
            Node child = scene.linkNodes.get(joint.child());
            jointPivots.put(joint.name(), child.getWorldTranslation().add(start));
            jointRotations.put(joint.name(), child.getWorldRotation().clone());
        }
        for (ImportedRobotScene.Part part : parts) {
            Node bodyNode = new Node("body-" + part.name());
            part.visual().removeFromParent();
            part.visual().setLocalTranslation(part.com().negate());
            part.visual().setLocalRotation(new Quaternion());
            bodyNode.attachChild(part.visual());
            field.attachChild(bodyNode);
            Vector3f center = part.origin().getTranslation().add(start)
                .add(part.origin().getRotation().mult(part.com()));
            RigidBodyControl body;
            if (part.name().equals(scene.urdf.rootLink)) {
                world.buildChassis(bodyNode, part.mass(), center, part.shape());
                body = world.chassisBody();
            } else {
                body = new RigidBodyControl(part.shape(), (float) part.mass());
                bodyNode.addControl(body);
                body.setPhysicsLocation(center);
                world.space().add(body);
            }
            body.setPhysicsRotation(part.origin().getRotation());
            body.setInverseInertiaLocal(new Vector3f(1f / part.inertia().x,
                1f / part.inertia().y, 1f / part.inertia().z));
            body.setEnableSleep(false);
            // Avoid double contacts in overlapping CAD assemblies; environment contact remains active.
            for (Body other : bodies.values()) body.addToIgnoreList(other.control());
            bodies.put(part.name(), new Body(part, body, bodyNode));
        }
        chassisNode = bodies.get(scene.urdf.rootLink).node();
        float assemblyYawInertia = 0;
        Vector3f chassisCenter = world.getChassisPosition();
        for (Body body : bodies.values()) {
            Vector3f offset = body.control().getPhysicsLocation().subtract(chassisCenter);
            Vector3f yawAxis = body.control().getPhysicsRotation().inverse().mult(Vector3f.UNIT_Y);
            Vector3f inertia = body.part().inertia();
            assemblyYawInertia += inertia.x * yawAxis.x * yawAxis.x + inertia.y * yawAxis.y * yawAxis.y
                + inertia.z * yawAxis.z * yawAxis.z + body.part().mass() * (offset.x * offset.x + offset.z * offset.z);
        }
        world.setDriveAssemblyProperties((float) scene.urdf.totalMassKg(), assemblyYawInertia);
        java.util.Set<String> usedMotors = new java.util.HashSet<>();
        for (RobotUrdf.Joint joint : scene.urdf.joints.values()) {
            if (joint.type().equals("fixed") || scene.wheelLinks.contains(joint.child())) continue;
            Body parent = bodies.get(scene.owners.get(joint.parent()));
            Body child = bodies.get(scene.owners.get(joint.child()));
            Axis axis = new Axis(joint, parent, child, jointPivots.get(joint.name()), jointRotations.get(joint.name()), world.space());
            for (RobotUrdf.Transmission tx : scene.urdf.transmissions.values()) {
                if (!tx.joint().equals(joint.name())) continue;
                axis.actuators.addAll(tx.actuators());
                for (RobotUrdf.Actuator actuator : tx.actuators()) {
                    SimDcMotorEx motor = scene.hardwareMap.tryGet(SimDcMotorEx.class, actuator.name());
                    if (motor != null) {
                        if (!usedMotors.add(actuator.name())) throw new IllegalArgumentException("Motor drives multiple physical joints: " + actuator.name());
                        motor.useExternalShaft();
                    }
                }
            }
            axes.add(axis);
        }
        world.space().addTickListener(this);
        System.out.println("[PHYSICS] Imported " + bodies.size() + " dynamic bodies and " + axes.size() + " mechanism joints; mass=" + scene.urdf.totalMassKg());
    }

    private final class Axis {
        final RobotUrdf.Joint joint;
        final Body parent, child;
        final Vector3f pivotParent, pivotChild, axisParent, axisChild;
        final Quaternion initialRelative;
        final List<RobotUrdf.Actuator> actuators = new ArrayList<>();
        double position, previousWrapped;

        Axis(RobotUrdf.Joint joint, Body parent, Body child, Vector3f pivot, Quaternion rotation, PhysicsSpace space) {
            this.joint = joint;
            this.parent = parent;
            this.child = child;
            Quaternion inverseParent = parent.control().getPhysicsRotation().inverse();
            Quaternion inverseChild = child.control().getPhysicsRotation().inverse();
            pivotParent = inverseParent.mult(pivot.subtract(parent.control().getPhysicsLocation()));
            pivotChild = inverseChild.mult(pivot.subtract(child.control().getPhysicsLocation()));
            Vector3f worldAxis = rotation.mult(ImportedRobotScene.position(joint.axis()).normalizeLocal());
            axisParent = inverseParent.mult(worldAxis);
            axisChild = inverseChild.mult(worldAxis);
            initialRelative = inverseParent.mult(child.control().getPhysicsRotation());
            if (joint.type().equals("prismatic")) {
                Quaternion frameWorld = new Quaternion().fromRotationMatrix(frameForX(worldAxis));
                SliderJoint slider = new SliderJoint(parent.control(), child.control(), pivotParent, pivotChild,
                    inverseParent.mult(frameWorld).toRotationMatrix(), inverseChild.mult(frameWorld).toRotationMatrix(), true);
                slider.setLowerLinLimit(joint.lower().floatValue());
                slider.setUpperLinLimit(joint.upper().floatValue());
                slider.setLowerAngLimit(0);
                slider.setUpperAngLimit(0);
                space.add(slider);
            } else {
                Quaternion frameWorld = new Quaternion().fromRotationMatrix(frameForX(worldAxis));
                New6Dof hinge = new New6Dof(parent.control(), child.control(), pivotParent, pivotChild,
                    inverseParent.mult(frameWorld).toRotationMatrix(), inverseChild.mult(frameWorld).toRotationMatrix(), RotationOrder.XYZ);
                for (int dof = 0; dof < 6; dof++) {
                    hinge.set(MotorParam.LowerLimit, dof, 0);
                    hinge.set(MotorParam.UpperLimit, dof, 0);
                }
                if (joint.lower() != null) {
                    if (joint.lower() < -Math.PI || joint.upper() > Math.PI)
                        throw new IllegalArgumentException("Hinge limits must be within [-pi, pi]: " + joint.name());
                    Quaternion original = child.control().getPhysicsRotation();
                    child.control().setPhysicsRotation(original.mult(new Quaternion().fromAngleAxis(0.01f, axisChild)));
                    float sign = Math.signum(hinge.getAngles(null).x);
                    child.control().setPhysicsRotation(original);
                    hinge.set(MotorParam.LowerLimit, 3, sign >= 0 ? joint.lower().floatValue() : -joint.upper().floatValue());
                    hinge.set(MotorParam.UpperLimit, 3, sign >= 0 ? joint.upper().floatValue() : -joint.lower().floatValue());
                } else {
                    hinge.set(MotorParam.LowerLimit, 3, 1);
                    hinge.set(MotorParam.UpperLimit, 3, -1); // lower > upper means free continuous rotation
                }
                space.add(hinge);
            }
        }

        double samplePosition() {
            if (joint.type().equals("prismatic")) {
                Vector3f a = parent.control().getPhysicsLocation().add(parent.control().getPhysicsRotation().mult(pivotParent));
                Vector3f b = child.control().getPhysicsLocation().add(child.control().getPhysicsRotation().mult(pivotChild));
                position = b.subtract(a).dot(parent.control().getPhysicsRotation().mult(axisParent));
            } else {
                Quaternion relative = parent.control().getPhysicsRotation().inverse().mult(child.control().getPhysicsRotation());
                Quaternion motion = initialRelative.inverse().mult(relative);
                double wrapped = 2 * Math.atan2(motion.getX() * axisChild.x + motion.getY() * axisChild.y
                    + motion.getZ() * axisChild.z, motion.getW());
                position += Math.atan2(Math.sin(wrapped - previousWrapped), Math.cos(wrapped - previousWrapped));
                previousWrapped = wrapped;
            }
            return position;
        }

        double rate(Vector3f axisWorld) {
            if (!joint.type().equals("prismatic"))
                return angularVelocity(child.control()).subtract(angularVelocity(parent.control())).dot(axisWorld);
            Vector3f ra = parent.control().getPhysicsRotation().mult(pivotParent);
            Vector3f rb = child.control().getPhysicsRotation().mult(pivotChild);
            Vector3f va = linearVelocity(parent.control()).add(angularVelocity(parent.control()).cross(ra));
            Vector3f vb = linearVelocity(child.control()).add(angularVelocity(child.control()).cross(rb));
            return vb.subtract(va).dot(axisWorld);
        }

        void drive(float dt) {
            double q = samplePosition();
            Vector3f worldAxis = parent.control().getPhysicsRotation().mult(axisParent);
            double velocity = rate(worldAxis);
            double effort = 0;
            double damping = 0;
            for (RobotUrdf.Actuator actuator : actuators) {
                SimDcMotorEx motor = scene.hardwareMap.tryGet(SimDcMotorEx.class, actuator.name());
                double reduction = actuator.mechanicalReduction();
                if (motor != null) {
                    motor.syncExternalShaft(q * reduction, velocity * reduction);
                    effort += motor.externalShaftTorque() * reduction;
                    damping += motor.externalTorqueDamping() * reduction * reduction;
                } else {
                    Servo servo = scene.hardwareMap.get(Servo.class, actuator.name());
                    double low = joint.lower() == null ? 0 : joint.lower();
                    double high = joint.upper() == null ? 2 * Math.PI : joint.upper();
                    double target = low + servo.getPosition() * (high - low) / reduction;
                    effort += Math.max(-2, Math.min(2, (target - q) * 10 - velocity));
                }
            }
            // Bound net motor impulse by effective joint inertia/mass to keep stiff gear reductions stable.
            double inverseEffective;
            if (joint.type().equals("prismatic")) {
                inverseEffective = 1 / child.part().mass() + (parent.control().isDynamic() ? 1 / parent.part().mass() : 0);
            } else {
                inverseEffective = worldAxis.dot(child.control().getInverseInertiaWorld(null).mult(worldAxis))
                    + (parent.control().isDynamic() ? worldAxis.dot(parent.control().getInverseInertiaWorld(null).mult(worldAxis)) : 0);
            }
            // Implicit back-EMF integration prevents high reductions from oscillating each step.
            if (joint.effort() != null) effort = Math.max(-joint.effort(), Math.min(joint.effort(), effort));
            effort /= 1 + dt * damping * inverseEffective;
            double cap = (joint.type().equals("prismatic") ? 50 : 100) / inverseEffective;
            effort = Math.max(-cap, Math.min(cap, effort));
            Vector3f impulse = worldAxis.mult((float) (effort * dt));
            if (joint.type().equals("prismatic")) {
                child.control().applyImpulse(impulse, child.control().getPhysicsRotation().mult(pivotChild));
                if (parent.control().isDynamic()) parent.control().applyImpulse(impulse.negate(), parent.control().getPhysicsRotation().mult(pivotParent));
            } else {
                child.control().applyTorqueImpulse(impulse);
                if (parent.control().isDynamic()) parent.control().applyTorqueImpulse(impulse.negate());
            }
        }
    }

    private static Vector3f linearVelocity(RigidBodyControl body) {
        return body.isDynamic() ? body.getLinearVelocity() : new Vector3f();
    }
    private static Vector3f angularVelocity(RigidBodyControl body) {
        return body.isDynamic() ? body.getAngularVelocity() : new Vector3f();
    }

    private static com.jme3.math.Matrix3f frameForX(Vector3f x) {
        Vector3f reference = Math.abs(x.y) < 0.9 ? Vector3f.UNIT_Y : Vector3f.UNIT_Z;
        Vector3f z = x.cross(reference).normalizeLocal();
        Vector3f y = z.cross(x).normalizeLocal();
        return new com.jme3.math.Matrix3f().setColumn(0, x).setColumn(1, y).setColumn(2, z);
    }

    double jointPosition(String name) {
        return axes.stream().filter(a -> a.joint.name().equals(name)).findFirst().orElseThrow().samplePosition();
    }

    Map<String, Double> jointPositions() {
        Map<String, Double> values = new LinkedHashMap<>();
        for (Axis axis : axes) values.put(axis.joint.name(), axis.samplePosition());
        return values;
    }

    @Override public void prePhysicsTick(PhysicsSpace space, float dt) {
        for (Axis axis : axes) axis.drive(dt);
    }
    @Override public void physicsTick(PhysicsSpace space, float dt) { }
}
