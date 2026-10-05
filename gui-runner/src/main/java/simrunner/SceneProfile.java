package simrunner;
import java.nio.file.*;
import java.util.*;
import simcore.MiniJson;

/** Placement is separate from model physics; references pin exact reviewed revisions. */
final class SceneProfile {
    static Map<String,Object> apply(Path path,Map<String,Object> root)throws Exception {
        var scene=MiniJson.parseObject(Files.readString(path));
        if(FieldPackage.num(scene,"schema_version")!=1)throw new IllegalArgumentException("Unsupported scene profile");
        for(String kind:List.of("robot","field"))if(scene.get(kind+"_profile") instanceof String ref) {
            var model=new ModelProfile(path.getParent().resolve(ref),true);var identity=FieldPackage.map(scene.get(kind+"_identity"));
            if(!kind.equals(model.kind)||!identity.get("profile_id").equals(model.data.get("profile_id"))||!identity.get("revision_id").equals(model.data.get("revision_id")))throw new IllegalArgumentException("Scene model revision changed: "+kind+". Select a saved revision in the editor.");
            if(kind.equals("robot"))root.put("robot_model_profile",model.path.toString());
            else {root.put("field",new LinkedHashMap<>(Map.of("source","imported","package",model.path.toString(),"mode",scene.get("mode"))));if(model.runtime.containsKey("field_behavior"))root.putIfAbsent("field_behavior",model.runtime.get("field_behavior"));}
        }
        if(scene.get("field_profile")==null)root.put("field",Map.of("source","generic","mode",scene.get("mode"),"piece_set",scene.getOrDefault("piece_set","biobuzz")));
        root.put("robot_start_xyz_m",scene.get("robot_start_xyz_m"));root.put("robot_start_yaw_rad",scene.get("robot_start_yaw_rad"));root.put("scene_pieces",scene.get("pieces"));return root;
    }
    static void legacyPieces(Map<String,Object> layout,ImportedFieldScene field,PhysicsWorld world)throws Exception {
        if(layout.isEmpty())return;
        var originals=new LinkedHashMap<String,Map<String,Object>>();for(var i:field.field.instances)if("piece".equals(i.get("owner")))originals.put(FieldPackage.str(i,"id"),i);
        for(var p:world.gamePieces()){var value=layout.get(p.id());if(value==null||Boolean.FALSE.equals(FieldPackage.map(value).get("enabled")))world.removePiece(p.id());}
        for(var e:layout.entrySet()) {
            var pose=FieldPackage.map(e.getValue());if(Boolean.FALSE.equals(pose.get("enabled"))||world.gamePieces().stream().anyMatch(p->p.id().equals(e.getKey())))continue;
            String id=pose.getOrDefault("source_id",e.getKey()).toString();if(!originals.containsKey(id))throw new IllegalArgumentException("Missing scene piece template: "+id);
            var instance=new LinkedHashMap<>(originals.get(id));instance.put("id",e.getKey());instance.put("xyz_m",pose.get("xyz_m"));instance.put("rpy_rad",pose.get("rpy_rad"));field.buildPiece(instance);
        }
    }
    static void placePieces(Map<String,Object> positions,PhysicsWorld world)throws Exception {
        for(var e:positions.entrySet()) {
            if(Boolean.FALSE.equals(FieldPackage.map(e.getValue()).get("enabled")))continue;
            var piece=world.gamePieces().stream().filter(p->p.id().equals(e.getKey())).findFirst().orElseThrow(()->new IllegalArgumentException("Scene references missing game piece: "+e.getKey()));
            var pose=FieldPackage.transform(FieldPackage.map(e.getValue()));piece.body().setPhysicsLocation(pose.getTranslation());piece.body().setPhysicsRotation(pose.getRotation());world.rememberPieceStart(piece.id());
        }
    }
}
