package simrunner;

import simcore.MiniJson;
import java.nio.file.*;
import java.security.MessageDigest;
import java.util.*;

/** Integrity-checked prepared revision. Normal simulation accepts reviewed revisions only. */
final class ModelProfile {
    final Path path, directory;
    final Map<String,Object> data, receipt, runtime, parameters;
    final String name, kind, digest;
    ModelProfile(Path file, boolean requireReady) throws Exception {
        path=file.toRealPath();directory=path.getParent();
        if(Files.size(path)>20_000_000)throw new IllegalArgumentException("Model profile exceeds budget");
        data=MiniJson.parseObject(Files.readString(path));
        receipt=MiniJson.parseObject(Files.readString(directory.resolve("receipt.json")));
        if(FieldPackage.num(data,"schema_version")!=1 || FieldPackage.num(receipt,"schema_version")!=1)throw new IllegalArgumentException("Unsupported model revision");
        if(!hash(path).equals(receipt.get("profile_sha256")))throw new IllegalArgumentException("Model settings changed; prepare and review them in the model editor");
        var files=FieldPackage.map(receipt.get("files"));
        if(files.size()>10000)throw new IllegalArgumentException("Model asset budget exceeded");
        for(var e:files.entrySet())if(!hash(resolve(e.getKey())).equals(e.getValue()))throw new IllegalArgumentException("Prepared/source asset changed: "+e.getKey()+". Reimport the CAD to migrate settings.");
        if(requireReady && (!Boolean.TRUE.equals(receipt.get("ready")) || !"ready".equals(FieldPackage.map(data.get("review")).get("state")) || !FieldPackage.maps(FieldPackage.map(data.get("migration")).get("pending")).isEmpty()))
            throw new IllegalArgumentException("Model requires collision review. Open the model editor, resolve migration choices, and save a reviewed revision.");
        name=FieldPackage.str(data,"name");kind=FieldPackage.str(data,"model_kind");digest=FieldPackage.str(receipt,"effective_digest");
        runtime=FieldPackage.map(receipt.get("runtime"));parameters=FieldPackage.map(data.get("parameters"));
    }
    Path resolve(String file) throws Exception {
        Path p=directory.resolve(file).normalize();
        if(!p.startsWith(directory)||!Files.isRegularFile(p)||!p.toRealPath().startsWith(directory))throw new IllegalArgumentException("Model asset escapes/missing: "+file);
        return p;
    }
    Path artifact(String name) throws Exception {return resolve(FieldPackage.str(FieldPackage.map(receipt.get("artifacts")),name));}
    boolean legacyField(){return "biobuzz".equals(data.get("adapter"));}
    static String hash(Path path) throws Exception {
        var h=MessageDigest.getInstance("SHA-256");try(var in=Files.newInputStream(path)){byte[] b=new byte[65536];int n;while((n=in.read(b))!=-1)h.update(b,0,n);}
        return HexFormat.of().formatHex(h.digest());
    }
    void configure(ImportedRobotScene scene) {
        if(runtime.containsKey("rotating_wheels"))scene.rotatingWheels=RotatingWheelConfig.parse(FieldPackage.map(runtime.get("rotating_wheels")));
        if(runtime.containsKey("drive_contacts"))scene.driveContacts=DriveContactConfig.parse(FieldPackage.map(runtime.get("drive_contacts")));
        scene.collisionMarginM=(float)FieldPackage.num(parameters,"collision_margin_m");
        scene.modelMaterials=FieldPackage.map(runtime.get("model_materials"));
        scene.passiveJoints=FieldPackage.map(runtime.get("passive_joints"));
        scene.contactCompliance=Boolean.TRUE.equals(parameters.get("use_contact_compliance"));
        scene.lockChassisLevel=Boolean.TRUE.equals(parameters.get("chassis_lock_level"));
        scene.contactStiffness=(float)FieldPackage.num(parameters,"contact_stiffness_n_per_m");scene.contactDamping=(float)FieldPackage.num(parameters,"contact_damping_ns_per_m");
        if(runtime.containsKey("collision_omissions")) {
            var omissions=new LinkedHashMap<String,String>();FieldPackage.map(runtime.get("collision_omissions")).forEach((k,v)->omissions.put(k,v.toString()));scene.collisionOmissions=omissions;
        }
    }
}
