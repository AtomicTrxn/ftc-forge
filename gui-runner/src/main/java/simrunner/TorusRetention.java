package simrunner;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.ManifoldPoints;
import com.jme3.bullet.collision.PersistentManifolds;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;
import simcore.SimDcMotorEx;
import java.util.Set;

/**
 * Lumped rubber/torus compression omitted by the rigid collision meshes. A moving pinch
 * anchor engages after loaded paddle contact. A bounded spring transfers equal/opposite
 * impulses to piece and chassis; its work loads the shaft. All native collisions remain.
 */
final class TorusRetention implements PhysicsTickListener {
    enum State { FREE, DRAWING, SEATED, EJECTING }
    private final PhysicsWorld world;
    private final ArticulatedRobot robot;
    private final SimDcMotorEx motor;
    private final MotorIntakeConfig intake;
    private final TorusRetentionConfig spec;
    private final Set<Long> paddles;
    private PhysicsRigidBody piece;
    private Vector3f anchor, normalAnchor;
    private State state=State.FREE;
    private double dwell, cooldown;
    private float normalSign=1;
    int acquisitions, releases;
    double peakForceN, peakLoadNm, peakTorqueNm;

    TorusRetention(PhysicsWorld world, ArticulatedRobot robot, SimDcMotorEx motor,
            MotorIntakeConfig intake, TorusRetentionConfig spec, Set<Long> paddles) {
        this.world=world; this.robot=robot; this.motor=motor; this.intake=intake; this.spec=spec;
        this.paddles=Set.copyOf(paddles);
        robot.addReflectedShaftInertia(intake.motor(),spec.reflectedShaftInertiaKgM2());
        world.space().addTickListener(this);
    }
    State state(){return state;}
    Vector3f anchor(){return anchor==null?Vector3f.ZERO:anchor.clone();}
    boolean retained(){return state==State.SEATED;}
    boolean loadedContact() {
        if (piece==null) return false;
        for (long manifold:world.space().listManifoldIds()) {
            long a=PersistentManifolds.getBodyAId(manifold),b=PersistentManifolds.getBodyBId(manifold);
            if (!((a==piece.nativeId()&&paddles.contains(b)) || (b==piece.nativeId()&&paddles.contains(a)))) continue;
            for (long point:PersistentManifolds.listPointIds(manifold))
                if (ManifoldPoints.getDistance1(point)<=.001 && ManifoldPoints.getAppliedImpulse(point)>1e-6) return true;
        }
        return false;
    }
    @Override public void prePhysicsTick(PhysicsSpace space,float dt) {
        robot.setShaftLoad(intake.motor(),0);
        if (piece!=world.gamePieceBody()) {
            piece=world.gamePieceBody(); state=State.FREE; dwell=0; cooldown=0;
            if(piece!=null){piece.setContactStiffness((float)spec.contactStiffnessNPerM());piece.setContactDamping((float)spec.contactDampingNsPerM());}
        }
        if (piece==null || !space.contains(piece)) return;
        cooldown=Math.max(0,cooldown-dt);
        Vector3f local=world.getChassisRotation().inverse().mult(piece.getPhysicsLocation().subtract(world.robotPointWorld(Vector3f.ZERO)));
        double speed=motor.getOmegaRadS()*intake.shaftSign();
        double command=motor.signedCommandedPower()*intake.shaftSign();
        if (state==State.FREE) {
            // A nearby piece, coasting unpowered shaft or reverse contact cannot acquire.
            dwell=command>.05 && speed>=intake.minSpeedRadS() && cooldown==0 && loadedContact() ? dwell+dt : 0;
            if (dwell<spec.contactDwellS()) return;
            anchor=local.clone(); normalAnchor=world.getChassisRotation().inverse().mult(piece.getPhysicsRotation().mult(Vector3f.UNIT_Z));
            // A torus plane has two equivalent normals. Always follow the inward shaft
            // direction from its downward normal, including after a release flips the ring.
            normalSign=normalAnchor.y>0?-1:1;normalAnchor.multLocal(normalSign); state=State.DRAWING; acquisitions++; dwell=0;
        }
        if (command<-.05 && state!=State.EJECTING) state=State.EJECTING;
        if (local.distance(anchor)>spec.breakDistanceM()) {release(); return;}
        Vector3f oldAnchor=anchor.clone();
        Vector3f oldNormal=normalAnchor.clone();
        boolean moving=state==State.DRAWING && command>.05 && speed>=intake.minSpeedRadS()
            || state==State.EJECTING && command<-.05 && speed<=-intake.minSpeedRadS();
        if (moving) {
            Vector3f target=state==State.EJECTING?spec.exit():spec.seat();
            Vector3f delta=target.subtract(anchor);
            float distance=(float)(Math.abs(speed)*spec.travelMPerRad()*dt);
            anchor.addLocal(delta.length()<=distance?delta:delta.normalizeLocal().multLocal(distance));
            Vector3f targetNormal=state==State.EJECTING?Vector3f.UNIT_Y.clone():spec.seatNormal();
            if(state==State.EJECTING && targetNormal.dot(normalAnchor)<0)targetNormal.negateLocal();
            float angle=normalAnchor.angleBetween(targetNormal);
            Vector3f axis=normalAnchor.cross(targetNormal);
            if(axis.lengthSquared()<1e-8f && angle>.05f)axis=Vector3f.UNIT_Z.clone();
            if(axis.lengthSquared()>1e-8f) normalAnchor=new com.jme3.math.Quaternion().fromAngleAxis(Math.min(angle,(float)(Math.abs(speed)*spec.angularTravelRadPerRad()*dt)),axis.normalizeLocal()).mult(normalAnchor).normalizeLocal();
        }
        Vector3f position=world.robotPointWorld(anchor);
        // React at the piece center, the same world point where its force acts,
        // so damping cannot create a net force couple between the two bodies.
        Vector3f radius=piece.getPhysicsLocation().subtract(world.getChassisPosition());
        Vector3f chassisVelocity=world.chassisBody().getLinearVelocity().add(world.chassisBody().getAngularVelocity().cross(radius));
        Vector3f anchorVelocity=world.getChassisRotation().mult(anchor.subtract(oldAnchor)).divideLocal(dt);
        Vector3f error=position.subtract(piece.getPhysicsLocation());
        Vector3f relative=piece.getLinearVelocity().subtract(chassisVelocity.add(anchorVelocity));
        // Implicit damping of the reduced piece/chassis mass keeps the finite spring stable.
        double inverseMass=1/piece.getMass()+1/world.chassisBody().getMass();
        Vector3f force=error.mult((float)spec.stiffnessNPerM()).subtractLocal(relative.mult((float)spec.dampingNsPerM()));
        force.divideLocal((float)(1+dt*spec.dampingNsPerM()*inverseMass+dt*dt*spec.stiffnessNPerM()*inverseMass));
        if (force.length()>spec.maxForceN()) force.normalizeLocal().multLocal((float)spec.maxForceN());
        Vector3f impulse=force.mult(dt);
        piece.applyCentralImpulse(impulse);
        world.chassisBody().applyImpulse(impulse.negate(),radius);
        peakForceN=Math.max(peakForceN,force.length());
        Vector3f actualNormal=piece.getPhysicsRotation().mult(Vector3f.UNIT_Z).multLocal(normalSign);
        Vector3f desiredNormal=world.getChassisRotation().mult(normalAnchor);
        if(state==State.SEATED && desiredNormal.dot(actualNormal)<0)desiredNormal.negateLocal();
        Vector3f cross=actualNormal.cross(desiredNormal);
        float angle=actualNormal.angleBetween(desiredNormal);
        Vector3f angularVelocity=piece.getAngularVelocity().subtract(world.chassisBody().getAngularVelocity());
        angularVelocity.subtractLocal(actualNormal.mult(angularVelocity.dot(actualNormal)));
        Vector3f torque=cross.lengthSquared()>1e-10f?cross.normalizeLocal().multLocal((float)(angle*spec.angularStiffnessNmPerRad())):new Vector3f();
        torque.subtractLocal(angularVelocity.mult((float)spec.angularDampingNmsPerRad()));
        if(torque.length()>spec.maxTorqueNm())torque.normalizeLocal().multLocal((float)spec.maxTorqueNm());
        piece.applyTorqueImpulse(torque.mult(dt));world.chassisBody().applyTorqueImpulse(torque.mult(-dt));
        peakTorqueNm=Math.max(peakTorqueNm,torque.length());
        Vector3f normalMotion=world.getChassisRotation().mult(oldNormal.cross(normalAnchor)).divideLocal(dt);
        if (moving && Math.abs(speed)>1e-3) {
            double load=Math.max(0,(force.dot(anchorVelocity)+torque.dot(normalMotion))/Math.abs(speed));
            robot.setShaftLoad(intake.motor(),Math.copySign(load,motor.getOmegaRadS()));
            peakLoadNm=Math.max(peakLoadNm,load);
        }
        if (state==State.DRAWING && anchor.distance(spec.seat())<.001 && local.distance(spec.seat())<.025 && normalAnchor.angleBetween(spec.seatNormal())<.05) state=State.SEATED;
        if (state==State.EJECTING && anchor.distance(spec.exit())<.001 && local.distance(spec.exit())<.035) release();
    }
    private void release(){state=State.FREE;dwell=0;cooldown=.25;releases++;}
    @Override public void physicsTick(PhysicsSpace space,float dt) {}
}
