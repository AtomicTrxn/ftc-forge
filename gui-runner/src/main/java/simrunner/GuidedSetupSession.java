package simrunner;

import java.nio.file.*;
import java.security.MessageDigest;
import java.util.*;
import simcore.MiniJson;

/** Local workflow progress; model receipts, never page flags, decide model readiness. */
final class GuidedSetupSession {
    record Step(String kind, String page) {
        String key() { return kind.isEmpty() ? page : kind + ":" + page; }
        String label() { return (kind.isEmpty() ? "" : Character.toUpperCase(kind.charAt(0)) + kind.substring(1) + " · ") + switch(page) {
            case "start" -> "Choose setup"; case "source" -> "Choose model";
            case "scale" -> "Size and orientation"; case "parts" -> "Parts and movement";
            case "physics" -> "Physics assumptions"; case "motion" -> "Motion demo"; case "review" -> "Review and save";
            case "scene" -> "Arrange scene"; default -> "Finish and use";
        }; }
    }
    final Path file;
    final Map<String,Object> data;
    private record Cached(String stamp,ModelProfile model){}
    private final Map<String,Cached> verified=new HashMap<>();
    private String stamp(ModelProfile model) throws Exception {
        var out=new StringBuilder();for(Path p:List.of(model.path,model.directory.resolve("receipt.json")))out.append(Files.size(p)).append(':').append(Files.getLastModifiedTime(p));
        for(String ref:FieldPackage.map(model.receipt.get("files")).keySet()){Path p=model.directory.resolve(ref);out.append(Files.size(p)).append(':').append(Files.getLastModifiedTime(p));}return out.toString();
    }
    ModelProfile checked(String kind) throws Exception {
        Path path=modelPath(kind);var cache=verified.get(kind);
        if(cache!=null&&cache.model.path.equals(path.toRealPath())&&cache.stamp.equals(stamp(cache.model)))return cache.model;
        var model=new ModelProfile(path,false);verified.put(kind,new Cached(stamp(model),model));return model;
    }
    GuidedSetupSession(Path library) throws Exception {
        Files.createDirectories(library.resolve("sessions"));
        file=library.resolve("sessions").toRealPath().resolve(UUID.randomUUID()+".json");
        data=new LinkedHashMap<>(); data.put("schema_version",1); data.put("goal","");
        data.put("models",new LinkedHashMap<String,Object>()); data.put("acknowledgments",new LinkedHashMap<String,Object>());
        data.put("current","start"); data.put("scene",null); save();
    }
    GuidedSetupSession(Path file, boolean resume) throws Exception {
        this.file=file.toRealPath(); data=MiniJson.parseObject(Files.readString(file));
        if(FieldPackage.num(data,"schema_version")!=1)throw new IllegalArgumentException("Unsupported setup session version");
    }
    void save() throws Exception { ProfileIO.save(file,data); }
    String goal() { return data.get("goal").toString(); }
    Path project() { return data.get("project") instanceof String p ? Path.of(p) : null; }
    void start(String goal, Path project) throws Exception {
        if(!Set.of("robot","field","simulation").contains(goal))throw new IllegalArgumentException("Choose a setup goal");
        if(project!=null&&!Files.isRegularFile(project.resolve("sim.config")))throw new IllegalArgumentException("Choose a team project folder containing sim.config");
        data.put("goal",goal);data.put("project",project==null?null:project.toRealPath().toString());save();
    }
    private final Map<String,String> checkedSources=new HashMap<>();
    boolean sourceAvailable(String kind) {
        try{Path path=modelPath(kind);var p=profile(kind);if(!kind.equals(p.get("model_kind")))return false;var files=FieldPackage.map(FieldPackage.map(p.get("source")).get("files"));String token=hash(files);var stamp=new StringBuilder(token);
            for(String ref:files.keySet()){Path asset=path.getParent().resolve("source").resolve(ref).normalize();if(!asset.toRealPath().startsWith(path.getParent().resolve("source").toRealPath()))return false;stamp.append(Files.size(asset)).append(Files.getLastModifiedTime(asset));}
            String key=path.toRealPath()+stamp.toString();if(!key.equals(checkedSources.get(kind))){for(var entry:files.entrySet())if(!ModelProfile.hash(path.getParent().resolve("source").resolve(entry.getKey())).equals(entry.getValue()))return false;checkedSources.put(kind,key);}return true;
        }catch(Exception e){return false;}
    }
    String reference(String kind) { return (String)FieldPackage.map(data.get("models")).get(kind); }
    Path modelPath(String kind) { String ref=reference(kind);return ref==null||ref.equals("generic")?null:file.getParent().resolve(ref).normalize(); }
    Map<String,Object> profile(String kind) throws Exception {return MiniJson.parseObject(Files.readString(modelPath(kind)));}
    boolean ready(String kind) {try{return modelPath(kind)!=null&&checked(kind).kind.equals(kind)&&Boolean.TRUE.equals(checked(kind).receipt.get("ready"))&&"ready".equals(FieldPackage.map(checked(kind).data.get("review")).get("state"))&&FieldPackage.maps(FieldPackage.map(checked(kind).data.get("migration")).get("pending")).isEmpty();}catch(Exception e){return false;}}
    void select(String kind, Path model) throws Exception {
        var p=new ModelProfile(model,false);if(!kind.equals(p.kind))throw new IllegalArgumentException("Choose a "+kind+" model");
        String ref=file.getParent().relativize(p.path).toString();if(!Objects.equals(reference(kind),ref))data.put("scene_validation",null);
        FieldPackage.map(data.get("models")).put(kind,ref);save();
    }
    void genericField() throws Exception {if(!"generic".equals(reference("field")))data.put("scene_validation",null);FieldPackage.map(data.get("models")).put("field","generic");save();}
    List<Step> steps() {
        var out=new ArrayList<Step>();out.add(new Step("","start"));
        for(String kind:goal().equals("simulation")?List.of("robot","field"):goal().isEmpty()?List.<String>of():List.of(goal()))
            for(String page:kind.equals("robot")?List.of("source","scale","parts","physics","motion","review"):List.of("source","scale","parts","physics","review"))out.add(new Step(kind,page));
        if(goal().equals("simulation"))out.add(new Step("","scene"));out.add(new Step("","finish"));return out;
    }
    String token(Step step) throws Exception {
        if(step.page.equals("scene"))return sceneToken();
        var p=profile(step.kind);var subset=new LinkedHashMap<String,Object>();subset.put("source",p.get("source"));
        if(step.page.equals("scale")) {var params=FieldPackage.map(p.get("parameters"));for(String key:List.of("length_unit","up_axis","origin_xyz_m","origin_rpy_rad"))subset.put(key,params.get(key));}
        else if(step.page.equals("parts")) {
            var parts=new TreeMap<String,Object>();FieldPackage.map(p.get("entities")).forEach((name,value)->{
                var s=FieldPackage.map(FieldPackage.map(value).get("settings"));parts.put(name,Arrays.asList(s.get("role"),s.get("joint"),s.get("actuators")));});
            subset.put("parts",parts);subset.put("migration",p.get("migration"));subset.put("runtime",p.get("runtime"));
        } else for(String key:List.of("parameters","entities","runtime","migration"))subset.put(key,p.get(key));
        return hash(subset);
    }
    static String hash(Object value) throws Exception {
        return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(ProfileIO.json(ordered(value)).getBytes(java.nio.charset.StandardCharsets.UTF_8)));
    }
    private static Object ordered(Object value) {
        if(value instanceof Map<?,?> m){var out=new TreeMap<String,Object>();m.forEach((k,v)->out.put(k.toString(),ordered(v)));return out;}
        if(value instanceof List<?> l)return l.stream().map(GuidedSetupSession::ordered).toList();return value;
    }
    void acknowledge(Step step) throws Exception {FieldPackage.map(data.get("acknowledgments")).put(step.key(),token(step));save();}
    boolean complete(Step step) {
        try {
            if(step.page.equals("start"))return !goal().isEmpty()&&(project()==null||Files.isRegularFile(project().resolve("sim.config")));
            if(step.page.equals("finish"))return steps().stream().filter(s->!s.page.equals("finish")).allMatch(this::complete);
            if(step.page.equals("scene"))return sceneReady();
            if("generic".equals(reference(step.kind)))return true;
            if(step.page.equals("source"))return sourceAvailable(step.kind);
            if(ready(step.kind))return true;
            if(step.page.equals("review"))return false;
            if(step.page.equals("parts")&&!FieldPackage.maps(FieldPackage.map(profile(step.kind).get("migration")).get("pending")).isEmpty())return false;
            return token(step).equals(FieldPackage.map(data.get("acknowledgments")).get(step.key()));
        } catch(Exception e){return false;}
    }
    Step firstIncomplete() {return steps().stream().filter(s->!complete(s)).findFirst().orElse(new Step("","finish"));}
    String sceneToken() throws Exception {
        if(!(data.get("scene") instanceof String ref))return "";
        Path scene=file.getParent().resolve(ref);var content=MiniJson.parseObject(Files.readString(scene));
        var models=new ArrayList<String>();for(String kind:List.of("robot","field"))if(content.get(kind+"_profile") instanceof String p){var model=new ModelProfile(scene.getParent().resolve(p),true);models.add(ModelProfile.hash(model.path));}
        return hash(List.of("scene-checks-v1 / Minie-9.0.3",content,models));
    }
    boolean sceneReady() {try{return !sceneToken().isEmpty()&&sceneToken().equals(data.get("scene_validation"));}catch(Exception e){return false;}}
    Path scenePath() {return data.get("scene") instanceof String ref?file.getParent().resolve(ref).normalize():null;}
    void scene(Path path, boolean validated) throws Exception {
        data.put("scene",file.getParent().relativize(path.toRealPath()).toString());data.put("scene_validation",validated?sceneToken():null);save();
    }
    void current(Step step) throws Exception {data.put("current",step.key());save();}
}
