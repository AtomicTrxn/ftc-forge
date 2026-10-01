package simrunner;

import java.nio.file.*;
import java.util.*;
import simcore.MiniJson;

/** Selecting a model preserves the active scene's placements and pins its new revision. */
final class ProjectModelSelection {
    static void apply(Path project,ModelProfile model,String policy)throws Exception {
        Path configFile=project.resolve("sim.config");var original=MiniJson.parseObject(Files.readString(configFile));var config=new LinkedHashMap<>(original);
        Path sceneFile=null;Map<String,Object> originalScene=null;
        try {
            if(config.get("scene_profile") instanceof String ref){sceneFile=project.resolve(ref).toRealPath();originalScene=MiniJson.parseObject(Files.readString(sceneFile));var scene=new LinkedHashMap<>(originalScene);scene.put(model.kind+"_profile",sceneFile.getParent().relativize(model.path).toString());scene.put(model.kind+"_identity",Map.of("profile_id",model.data.get("profile_id"),"revision_id",model.data.get("revision_id")));ProfileIO.save(sceneFile,scene);}
            if(model.kind.equals("robot"))config.put("robot_model_profile",model.path.toString());
            else {var field=new LinkedHashMap<String,Object>(config.containsKey("field")?FieldPackage.map(config.get("field")):Map.of("mode","field-only"));field.put("source","imported");field.put("package",model.path.toString());config.put("field",field);}
            if(model.kind.equals("robot")||!config.containsKey("model_settings"))config.put("model_settings",policy);ProfileIO.save(configFile,config);SimConfig.load(project);
        }catch(Exception error){ProfileIO.save(configFile,original);if(sceneFile!=null&&originalScene!=null)ProfileIO.save(sceneFile,originalScene);throw error;}
    }
}
