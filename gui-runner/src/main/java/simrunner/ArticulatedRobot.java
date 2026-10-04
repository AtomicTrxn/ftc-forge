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
import physics.ServoModel;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Finite-effort, environment-collidable URDF mechanisms with physical encoder feedback. */
final class ArticulatedRobot implements PhysicsTickListener {
    private record Body(ImportedRobotScene.Part part, RigidBodyControl control, Node node) { }
    private final ImportedRobotScene scene;
    private final Map<String, ServoModel.Spec> servoSpecs;
    private final Map<String, Body> bodies = new LinkedHashMap<>();
    private final Map<String, Double> attachedInertia = new LinkedHashMap<>();
    private final Map<String, Double> shaftLoads = new LinkedHashMap<>();
    void setShaftLoad(String motor, double torqueNm) { shaftLoads.put(motor, torqueNm); }
    void addReflectedShaftInertia(String motorName,double moment) {
        Axis axis=axes.stream().filter(a->a.actuators.stream().anyMatch(t->t.name().equals(motorName))).findFirst().orElseThrow();
        double reduction=axis.actuators.stream().filter(a->a.name().equals(motorName)).findFirst().orElseThrow().mechanicalReduction();
        moment*=reduction*reduction;
        Vector3f inverse=axis.child.control().getInverseInertiaLocal(null);
        Vector3f a=axis.axisChild;
        // Diagonal projection in the CAD principal frame. REV shaft axes align with its
        // principal axis; reject arbitrary off-axis configurations rather than lose products.
        if(Math.max(Math.abs(a.x),Math.max(Math.abs(a.y),Math.abs(a.z)))<.999f)
            throw new IllegalArgumentException("Reflected shaft inertia requires a principal-axis-aligned rotor");
        axis.child.control().setInverseInertiaLocal(new Vector3f((float)(1/(1/inverse.x+moment*a.x*a.x)),
            (float)(1/(1/inverse.y+moment*a.y*a.y)),(float)(1/(1/inverse.z+moment*a.z*a.z))));
        addAttachedInertia(axis.child.part().name(),moment);
    }
    private final List<Axis> axes = new ArrayList<>();
    record DiagnosticJoint(String name,String body,String type,double position,double velocity,double effort,boolean effortIsLimit,Double lower,Double upper,Vector3f pivot,Vector3f axis) { }
    Map<Long,String> diagnosticBodies() {
        var names=new LinkedHashMap<Long,String>();bodies.forEach((name,body)->names.put(body.control().nativeId(),name));return Map.copyOf(names);
    }
    List<DiagnosticJoint> diagnosticJoints() {
        // Read cached last-tick values. Do not sample/unroll joint position from a renderer.
        return axes.stream().limit(32).map(a->new DiagnosticJoint(a.joint.name(),a.child.part().name(),a.joint.type(),a.position,a.lastVelocity,a.lastEffort,a.effortIsLimit,a.joint.lower(),a.joint.upper(),
            a.parent.control().getPhysicsLocation().add(a.parent.control().getPhysicsRotation().mult(a.pivotParent)),a.parent.control().getPhysicsRotation().mult(a.axisParent))).toList();
    }
    int diagnosticJointCount(){return axes.size();}
    final Node chassisNode;

    ArticulatedRobot(ImportedRobotScene scene, PhysicsWorld world, Node field, Vector3f start) throws Exception {
        this(scene, world, field, start, Map.of());
    }

    ArticulatedRobot(ImportedRobotScene scene, PhysicsWorld world, Node field, Vector3f start,
                     Map<String, ServoModel.Spec> servoSpecs) throws Exception {
        this(scene,world,field,start,servoSpecs,new Quaternion());
    }
    ArticulatedRobot(ImportedRobotScene scene, PhysicsWorld world, Node field, Vector3f start,
                     Map<String, ServoModel.Spec> servoSpecs, Quaternion placement) throws Exception {
        this.scene = scene;
        this.servoSpecs = servoSpecs;
        for (RobotUrdf.Transmission tx : scene.passiveConstruction ? List.<RobotUrdf.Transmission>of() : scene.urdf.transmissions.values()) {
            if (scene.isDriveWheel(scene.urdf.joints.get(tx.joint()))) continue;
            for (RobotUrdf.Actuator actuator : tx.actuators()) {
                if (scene.hardwareMap.tryGet(Servo.class, actuator.name()) != null
                    && !servoSpecs.containsKey(actuator.name()))
                    throw new IllegalArgumentException("Missing servoPhysics entry for URDF actuator " + actuator.name());
            }
        }
        List<ImportedRobotScene.Part> parts = scene.parts();
        // Snapshot joint origins before detaching/reparenting the visual tree.
        Map<String, Vector3f> jointPivots = new LinkedHashMap<>();
        Map<String, Quaternion> jointRotations = new LinkedHashMap<>();
        for (RobotUrdf.Joint joint : scene.urdf.joints.values()) {
            Node child = scene.linkNodes.get(joint.child());
            jointPivots.put(joint.name(), placement.mult(child.getWorldTranslation()).add(start));
            jointRotations.put(joint.name(), placement.mult(child.getWorldRotation()));
        }
        for (ImportedRobotScene.Part part : parts) {
            Node bodyNode = new Node("body-" + part.name());
            part.visual().removeFromParent();
            Quaternion principalInverse = part.principalRotation().inverse();
            part.visual().setLocalTranslation(principalInverse.mult(part.com().negate()));
            part.visual().setLocalRotation(principalInverse);
            bodyNode.attachChild(part.visual());
            field.attachChild(bodyNode);
            Vector3f center = placement.mult(part.origin().getTranslation()
                .add(part.origin().getRotation().mult(part.com()))).add(start);
            RigidBodyControl body;
            if (part.name().equals(scene.urdf.rootLink)) {
                world.buildChassis(bodyNode, part.mass(), center, part.shape());
                body = world.chassisBody();
                if(scene.wheelContactsEnabled())body.setFriction(.5f);
            } else {
                body = new RigidBodyControl(part.shape(), (float) part.mass());
                bodyNode.addControl(body);
                body.setPhysicsLocation(center);
                world.space().add(body);
            }
            body.setPhysicsRotation(placement.mult(part.origin().getRotation()).mult(part.principalRotation()));
            body.setInverseInertiaLocal(new Vector3f(1f / part.inertia().x,
                1f / part.inertia().y, 1f / part.inertia().z));
            body.setEnableSleep(false);
            if(scene.contactCompliance){body.setContactStiffness(scene.contactStiffness);body.setContactDamping(scene.contactDamping);}
            Object material=scene.modelMaterials.get(part.name());
            if(material==null) material=scene.owners.entrySet().stream().filter(e->e.getValue().equals(part.name())&&scene.modelMaterials.containsKey(e.getKey())).map(e->scene.modelMaterials.get(e.getKey())).findFirst().orElse(null);
            if(material!=null){var m=FieldPackage.map(material);body.setFriction((float)FieldPackage.num(m,"friction"));body.setRestitution((float)FieldPackage.num(m,"restitution"));body.setRollingFriction((float)FieldPackage.num(m,"rolling_friction"));body.setSpinningFriction((float)FieldPackage.num(m,"spinning_friction"));}
            bodies.put(part.name(), new Body(part, body, bodyNode));
        }
        Body chassis = bodies.get(scene.urdf.rootLink);
        chassisNode = chassis.node();
        world.setRobotFrame(chassis.part().principalRotation().inverse(),
            chassis.part().principalRotation().inverse().mult(chassis.part().com().negate()));
        if(scene.lockChassisLevel)world.constrainChassisLevel();else world.chassisBody().setAngularFactor(1);
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
        if(scene.wheelContactsEnabled())world.installDriveContacts(new DriveContacts(world,scene.driveContacts,chassis.part().contactLinks(),scene.wheelJoints));
        java.util.Set<String> usedMotors = new java.util.HashSet<>();
        for (RobotUrdf.Joint joint : scene.urdf.joints.values()) {
            if (joint.type().equals("fixed") || scene.wheelLinks.contains(joint.child())) continue;
            Body parent = bodies.get(scene.owners.get(joint.parent()));
            Body child = bodies.get(scene.owners.get(joint.child()));
            // Adjacent bodies share a joint and often have deliberately overlapping CAD geometry.
            // Nonadjacent links and siblings keep full collision response.
            parent.control().addToIgnoreList(child.control());
            Axis axis = new Axis(joint, parent, child, jointPivots.get(joint.name()), jointRotations.get(joint.name()), world.space());
            for (RobotUrdf.Transmission tx : scene.passiveConstruction ? List.<RobotUrdf.Transmission>of() : scene.urdf.transmissions.values()) {
                if (!tx.joint().equals(joint.name())) continue;
                axis.actuators.addAll(tx.actuators());
                for (RobotUrdf.Actuator actuator : tx.actuators()) {
                    SimDcMotorEx motor = scene.hardwareMap.tryGet(SimDcMotorEx.class, actuator.name());
                    if (motor != null) {
                        if (!usedMotors.add(actuator.name())) throw new IllegalArgumentException("Motor drives multiple physical joints: " + actuator.name());
                        motor.useExternalShaft();
                    } else if (!servoSpecs.containsKey(actuator.name())) {
                        throw new IllegalArgumentException("Missing servoPhysics entry for URDF actuator " + actuator.name());
                    }
                }
            }
            axes.add(axis);
        }
        for (Axis follower : axes) {
            if (follower.joint.mimic() == null) continue;
            Axis source = axes.stream().filter(a -> a.joint.name().equals(follower.joint.mimic())).findFirst()
                .orElseThrow(() -> new IllegalArgumentException("Mimic source must be a physical mechanism"));
            Vector3f a = source.parent.control().getPhysicsRotation().mult(source.axisParent);
            Vector3f b = follower.parent.control().getPhysicsRotation().mult(follower.axisParent);
            if (a.dot(b) < .999f || follower.joint.multiplier() != 1)
                throw new IllegalArgumentException("Physical mimic supports 1:1 parallel shafts with matching axis directions");
            if (scene.flexibleIntake != null) {
                // GearJoint constrains angular velocity and permits accumulated phase drift under
                // contact load. Matching continuous shafts can instead lock relative orientation
                // while leaving their translations free: an ideal 1:1 chain position constraint.
                Quaternion frame = new Quaternion().fromRotationMatrix(frameForX(a));
                New6Dof chain = New6Dof.newInstance(source.child.control(), follower.child.control(),
                    source.child.control().getPhysicsLocation(), frame, RotationOrder.XYZ);
                for(int dof=0;dof<6;dof++) {
                    chain.set(MotorParam.LowerLimit,dof,dof<3?1:0);
                    chain.set(MotorParam.UpperLimit,dof,dof<3?-1:0);
                }
                world.space().add(chain);
            } else world.space().add(new com.jme3.bullet.joints.GearJoint(source.child.control(), follower.child.control(),
                source.axisChild, follower.axisChild, (float) (-1 / follower.joint.multiplier())));
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
        double lastVelocity,lastEffort;
        boolean effortIsLimit;
        New6Dof hinge;
        float motorAngleSign=1;

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
                hinge = new New6Dof(parent.control(), child.control(), pivotParent, pivotChild,
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
                Quaternion original = child.control().getPhysicsRotation();
                child.control().setPhysicsRotation(original.mult(new Quaternion().fromAngleAxis(.01f, axisChild)));
                motorAngleSign = Math.signum(hinge.getAngles(null).x);
                child.control().setPhysicsRotation(original);
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
            lastVelocity=velocity;
            var passive=scene.passiveJoints.containsKey(joint.name())?FieldPackage.map(scene.passiveJoints.get(joint.name())):null;
            boolean linear=joint.type().equals("prismatic");
            double effort = passive==null?0:-FieldPackage.num(passive,linear?"joint_spring_n_per_m":"joint_spring_nm_per_rad")*(q-FieldPackage.num(passive,linear?"joint_rest_m":"joint_rest_rad"))-FieldPackage.num(passive,linear?"joint_damping_ns_per_m":"joint_damping_nm_s")*velocity;
            double damping = 0;
            for (RobotUrdf.Actuator actuator : actuators) {
                SimDcMotorEx motor = scene.hardwareMap.tryGet(SimDcMotorEx.class, actuator.name());
                double reduction = actuator.mechanicalReduction();
                if (motor != null) {
                    motor.syncExternalShaft(q * reduction, velocity * reduction);
                    effort += (motor.externalShaftTorque() - shaftLoads.getOrDefault(actuator.name(),0d)) * reduction;
                    damping += motor.externalTorqueDamping() * reduction * reduction;
                } else {
                    Servo servo = scene.hardwareMap.get(Servo.class, actuator.name());
                    ServoModel.Spec spec = servoSpecs.get(actuator.name());
                    if (spec == null) throw new IllegalArgumentException(
                        "Missing servoPhysics entry for URDF actuator " + actuator.name());
                    double position = servo.getDirection() == Servo.Direction.REVERSE
                        ? 1 - servo.getPosition() : servo.getPosition();
                    double low = joint.lower() == null ? 0 : joint.lower();
                    double target = low + position * spec.travelRad() / reduction;
                    if (joint.lower() != null) target = Math.max(joint.lower(), Math.min(joint.upper(), target));
                    effort += ServoModel.effort(spec, (target - q) * reduction, velocity * reduction) * reduction;
                }
            }
            // Bound net motor impulse by effective joint inertia/mass to keep stiff gear reductions stable.
            double inverseEffective;
            if (joint.type().equals("prismatic")) {
                inverseEffective = 1 / child.part().mass() + (parent.control().isDynamic() ? 1 / parent.part().mass() : 0);
            } else {
                inverseEffective = 1 / rotatingInertia(this)
                    + (parent.control().isDynamic() ? worldAxis.dot(parent.control().getInverseInertiaWorld(null).mult(worldAxis)) : 0);
            }
            // Implicit back-EMF integration prevents high reductions from oscillating each step.
            if (joint.effort() != null) effort = Math.max(-joint.effort(), Math.min(joint.effort(), effort));
            effort /= 1 + dt * damping * inverseEffective;
            boolean elasticAxis = hinge != null && scene.flexibleIntake != null &&
                (attachedInertia.containsKey(child.part().name()) || axes.stream().anyMatch(a -> joint.name().equals(a.joint.mimic()) && attachedInertia.containsKey(a.child.part().name())));
            if (elasticAxis && !actuators.isEmpty()) {
                // Solve motor effort together with flap/contact constraints. Applying a torque
                // impulse to the tiny rigid core before solving contacts required an artificial
                // acceleration cap. The native motor instead bounds torque in the solve.
                hinge.getRotationMotor(0).setMotorEnabled(true);
                hinge.set(MotorParam.TargetVelocity, 3, (float)((velocity+effort*dt*inverseEffective)*motorAngleSign));
                hinge.set(MotorParam.MaxMotorForce, 3, (float)Math.abs(effort));
                lastEffort=(float)Math.abs(effort);
                effortIsLimit=true;
                return;
            }
            double cap = (joint.type().equals("prismatic") ? 50 : 100) / inverseEffective;
            effort = Math.max(-cap, Math.min(cap, effort));
            lastEffort=effort;
            effortIsLimit=false;
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

    void addAttachedInertia(String owner, double moment) { attachedInertia.merge(owner, moment, Double::sum); }

    private double rotatingInertia(Axis axis) {
        Vector3f worldAxis = axis.parent.control().getPhysicsRotation().mult(axis.axisParent);
        double inertia = 1 / worldAxis.dot(axis.child.control().getInverseInertiaWorld(null).mult(worldAxis))
            + attachedInertia.getOrDefault(axis.child.part().name(), 0.0);
        for (Axis follower : axes) if (axis.joint.name().equals(follower.joint.mimic()))
            inertia += rotatingInertia(follower);
        return inertia;
    }

    RigidBodyControl bodyForLink(String name) { return bodies.get(scene.owners.get(name)).control(); }

    com.jme3.math.Transform linkFrameWorld(String name) {
        Body owner = bodies.get(scene.owners.get(name));
        com.jme3.math.Transform local = new com.jme3.math.Transform();
        com.jme3.scene.Spatial node = scene.linkNodes.get(name);
        while (node != owner.node()) {
            local.combineWithParent(node.getLocalTransform());
            node = node.getParent();
        }
        return local.combineWithParent(new com.jme3.math.Transform(owner.control().getPhysicsLocation(), owner.control().getPhysicsRotation()));
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
