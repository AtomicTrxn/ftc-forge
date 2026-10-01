package simrunner;

import com.jme3.asset.DesktopAssetManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import simcore.RobotUrdf;
import java.nio.file.Path;
import java.util.Set;

/** Saves coverage findings even when an import cannot start physics. No TeamCode runs. */
public final class CollisionAuditCli {
    public static void main(String[] args)throws Exception {
        if(args.length!=2)throw new IllegalArgumentException("Usage: CollisionAuditCli <robotProject> <report.json>");
        Path project=Path.of(args[0]);var config=SimConfig.load(project);
        if(config.urdf==null)throw new IllegalArgumentException("Collision audit requires a robot URDF in sim.config");
        Path source=project.resolve(config.urdf);
        var drive=config.drive==null?Set.of("left_front_drive","right_front_drive","left_back_drive","right_back_drive"):Set.copyOf(config.drive.motorNames());
        var scene=new ImportedRobotScene(RobotUrdf.parse(source),source,new HardwareMap(),new DesktopAssetManager(true),config.vhacdMaxHulls,drive,.0005f);
        scene.tireContacts=config.tires!=null;scene.flexibleIntake=config.flexibleIntake;scene.collisionOmissions=config.collisionOmissions;
        var audit=CollisionAudit.inspect(scene);audit.write(Path.of(args[1]),source);
        System.out.println("[COLLISION AUDIT] "+audit.summary()+" | report="+Path.of(args[1]).toAbsolutePath());
        audit.requireUsable();
    }
}
