package simrunner;

import java.nio.file.*;
import java.util.*;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.bounding.BoundingBox;
import com.qualcomm.robotcore.hardware.*;
import simcore.*;

/** All wizard operations are usable without Swing and run on its worker thread. */
final class GuidedSetupController {
    final ModelEditorController models;
    GuidedSetupSession session;
    private Process previewProcess,motionProcess;private Path previewPath;Path lastLog;
    private String motionPreviousRun="";
    GuidedSetupController(Path library, Path project) throws Exception {
        models=new ModelEditorController(library);session=new GuidedSetupSession(library);
        if(project!=null)session.data.put("project",project.toRealPath().toString());session.save();
    }
    void resume(Path file) throws Exception {
        var restored=new GuidedSetupSession(file,true);session=restored;
        for(String kind:List.of("robot","field")){Path p=restored.modelPath(kind);if(p!=null&&Files.isRegularFile(p)&&!p.toString().contains("/revisions/"))try{models.run("compile",p.toString());}catch(Exception ignored){/* Retain invalid drafts for guided correction. */}}
        session=restored;session.current(session.firstIncomplete());
    }
    void load(String kind, String operation, Path input, Path reuse) throws Exception {
        Object result=switch(operation) {
            case "saved" -> input;
            case "bundle" -> models.run("import-profile",input.toString(),models.library.toString());
            case "fresh" -> models.run("import",input.toString(),models.library.toString(),"--kind",kind,"--fresh");
            case "capture" -> models.run("capture-"+kind,input.toString(),models.library.toString());
            default -> reuse==null?models.run("import",input.toString(),models.library.toString(),"--kind",kind):models.run("import",input.toString(),models.library.toString(),"--kind",kind,"--reuse",reuse.toString());
        };
        Path p=Path.of(result.toString());var model=new ModelProfile(p,false);
        if(!kind.equals(model.kind))throw new IllegalArgumentException("This is a "+model.kind+" model. Select a "+kind+" source.");
        if(Boolean.TRUE.equals(model.receipt.get("ready"))&&!p.toString().contains("/revisions/"))p=Path.of(models.run("save",p.toString(),models.library.toString(),"--reviewed").toString());
        validateRuntime(p);session.select(kind,p);
    }
    void edit(String kind) throws Exception {
        Path p=session.modelPath(kind);
        if(p.toString().contains("/revisions/"))p=Path.of(models.run("edit",p.toString(),models.library.toString()).toString());
        var data=MiniJson.parseObject(Files.readString(p));data.put("review",Map.of("state","draft"));ProfileIO.save(p,data);
        try{models.run("compile",p.toString());}catch(Exception ignored){/* Advanced editor can repair an invalid draft. */}
        FieldPackage.map(session.data.get("models")).put(kind,session.file.getParent().relativize(p.toRealPath()).toString());session.data.put("scene_validation",null);session.save();
    }
    void update(String kind, Map<String,Object> profile) throws Exception {
        Path p=session.modelPath(kind);
        if(p.toString().contains("/revisions/"))p=Path.of(models.run("edit",p.toString(),models.library.toString()).toString());
        profile.put("review",Map.of("state","draft"));ProfileIO.save(p,profile);
        FieldPackage.map(session.data.get("models")).put(kind,session.file.getParent().relativize(p.toRealPath()).toString());session.save();
    }
    void compile(String kind) throws Exception {if(!session.ready(kind))models.run("compile",session.modelPath(kind).toString());validateRuntime(session.modelPath(kind));}
    void wheelSupport(boolean enabled) throws Exception {
        var profile=session.profile("robot");var runtime=FieldPackage.map(profile.get("runtime"));
        runtime.putIfAbsent("drive_contacts",DriveContactConfig.defaults(enabled));
        FieldPackage.map(runtime.get("drive_contacts")).put("enabled",enabled);
        DriveContactConfig.parse(FieldPackage.map(runtime.get("drive_contacts")));
        FieldPackage.map(profile.get("provenance")).put("runtime/drive_contacts/enabled","user supplied: wheel support mode");
        update("robot",profile);compile("robot");
    }
    void measurements(RobotMeasurements.Entry entry) throws Exception {
        if(entry.empty())return;
        var measured=RobotMeasurements.apply(session.profile("robot"),entry);
        update("robot",measured);
        session.data.put("scene_validation",null);session.data.remove("project_check");session.data.remove("opmodes");session.save();
        compile("robot");
    }
    Path motionReport(){return session.file.getParent().resolve("motion-demos").resolve(session.file.getFileName());}
    void continueAfterMotion() throws Exception {session.acknowledge(new GuidedSetupSession.Step("robot","motion"));}
    RobotMotionDemo.Setup motionSetup() throws Exception {compile("robot");return RobotMotionDemo.setup(session.modelPath("robot"),session.project());}
    void motionDemo() throws Exception {motionDemo("");}
    void motionDemo(String group) throws Exception {
        if(motionProcess!=null&&motionProcess.isAlive()) {
            var report=Files.isRegularFile(motionReport())?MiniJson.parseObject(Files.readString(motionReport())):Map.<String,Object>of();
            if(!completedNewMotionReport(motionPreviousRun,report))throw new IllegalArgumentException("A motion demo is still running. Close it with Esc before launching a retest, or use R to replay that window.");
            motionProcess.destroy();
            if(!motionProcess.waitFor(3,java.util.concurrent.TimeUnit.SECONDS))throw new IllegalArgumentException("Close the previous demo with Esc before launching a retest.");
        }
        prepareMotionRetest(group);
        motionPreviousRun=Files.isRegularFile(motionReport())?String.valueOf(MiniJson.parseObject(Files.readString(motionReport())).getOrDefault("run_id","")):"";
        var args=new ArrayList<>(List.of(session.modelPath("robot").toString(),session.project()==null?"-":session.project().toString(),motionReport().toString()));if(!group.isEmpty())args.addAll(List.of("--only",group));
        motionProcess=ModelEditorController.javaProcess(RobotMotionDemoApp.class,args,true);
    }
    static boolean completedNewMotionReport(String previous,Map<String,Object> report) {
        return report.get("run_id") instanceof String run&&!run.isBlank()&&!run.equals(previous)&&Boolean.TRUE.equals(report.get("complete"))&&"finished".equals(report.get("status"));
    }
    void prepareMotionRetest(String group) throws Exception {
        var setup=motionSetup();if(setup.plan().empty())throw new IllegalArgumentException(setup.plan().summary());
        if(!group.isEmpty()&&setup.plan().items().stream().noneMatch(i->i.group().equals(group)))throw new IllegalArgumentException("This movement is no longer configured. Choose a current movement to retest.");
        if(!RobotMotionReview.entries(setup.profile().data).isEmpty())persistMotionReview(RobotMotionReview.requireRetest(setup.profile().data,group));
    }
    RobotMotionReview.Snapshot motionReview() throws Exception {return RobotMotionReview.read(motionSetup(),Files.isRegularFile(motionReport())?MiniJson.parseObject(Files.readString(motionReport())):Map.of());}
    void saveMotionReviews(RobotMotionReview.Snapshot expected,Map<String,String> choices,Map<String,String> notes)throws Exception {
        if(choices.isEmpty())return;
        var current=motionReview();if(!expected.setup().context().equals(current.setup().context()))throw new IllegalArgumentException("Model or hardware changed. Reopen the movement review and retest before saving.");
        for(String id:choices.keySet()) {
            var before=expected.rows().stream().filter(r->r.item().id().equals(id)).findFirst().orElseThrow();
            var now=current.rows().stream().filter(r->r.item().id().equals(id)).findFirst().orElseThrow();
            if(!before.runId().equals(now.runId())||!GuidedSetupSession.hash(before.observation()).equals(GuidedSetupSession.hash(now.observation())))throw new IllegalArgumentException("A newer demo changed these observations. Reopen the movement review before saving.");
        }
        persistMotionReview(RobotMotionReview.assess(current,choices,notes));
    }
    private void persistMotionReview(Map<String,Object> review)throws Exception {
        boolean ready=session.ready("robot");Path path=session.modelPath("robot");
        if(path.toString().contains("/revisions/"))path=Path.of(models.run("edit",path.toString(),models.library.toString()).toString());
        var data=MiniJson.parseObject(Files.readString(path));data.put("motion_review",review);ProfileIO.save(path,data);models.run("compile",path.toString());
        if(ready)path=Path.of(models.run("save",path.toString(),models.library.toString(),"--reviewed").toString());
        session.select("robot",path);
    }
    String motionResults() throws Exception {
        if(!Files.isRegularFile(motionReport()))return "No demo observations yet. Run the demo, then return here to view results.";
        var report=MiniJson.parseObject(Files.readString(motionReport()));
        if(report.containsKey("error"))return "Last demo needs attention: "+report.get("error")+"\nCorrect the setup and replay to obtain current observations.";
        if(!Objects.equals(report.get("context"),motionSetup().context()))return "Model or project hardware changed since these observations. Replay the demo for the current setup.";
        var text=new StringBuilder("Status: "+report.get("status")+"\n"+report.get("hardware")+"\n");
        for(var row:FieldPackage.maps(report.get("observations"))) {
            text.append("\n").append(row.get("action")).append(": ").append(row.get("outcome")).append("\n");
            if(row.containsKey("forward_m"))text.append(String.format(java.util.Locale.ROOT,"Forward %.3f m · left %.3f m · turn %.1f degrees\n",FieldPackage.num(row,"forward_m"),FieldPackage.num(row,"left_m"),Math.toDegrees(FieldPackage.num(row,"yaw_rad"))));
            if(row.containsKey("start"))text.append(String.format(java.util.Locale.ROOT,"Joint %.3f → %.3f %s · target %.3f %s\n",FieldPackage.num(row,"start"),FieldPackage.num(row,"end"),row.get("unit"),FieldPackage.num(row,"target"),row.get("unit")));
            text.append(row.get("detail")).append("\n");
        }
        for(var note:(List<?>)report.getOrDefault("notes",List.of()))text.append("\n").append(note);
        text.append("\n\nCompare observed movements with your physical robot. Return to Parts / Physics assumptions to correct mismatches. Demo observations do not approve collisions or establish physical accuracy.");return text.toString();
    }
    private void validateRuntime(Path path) throws Exception {
        var model=new ModelProfile(path,false);if(!model.kind.equals("robot"))return;
        var root=new LinkedHashMap<>(model.runtime);root.put("urdf",model.artifact("robot").toString());var config=SimConfig.parse(model.directory,root);
        if(config.totalMassKg!=null&&(!Double.isFinite(config.totalMassKg)||config.totalMassKg<=0))throw new IllegalArgumentException("Total robot mass must be positive and finite.");
    }
    void units(String kind, String unit) throws Exception {
        if(session.ready(kind))edit(kind);models.run("units",session.modelPath(kind).toString(),unit);
    }
    void resolve(String kind, int index, String choice) throws Exception {models.run("resolve",session.modelPath(kind).toString(),Integer.toString(index),choice);session.save();}
    Path viewFile(){return session.file.resolveSibling(session.file.getFileName()+"-view.json");}
    void focus(String link) throws Exception {ProfileIO.save(viewFile(),Map.of("link",link,"nonce",UUID.randomUUID().toString()));}
    void preview(String kind) throws Exception {
        compile(kind);Path path=session.modelPath(kind);if(previewProcess!=null&&previewProcess.isAlive()&&path.equals(previewPath))return;if(previewProcess!=null&&previewProcess.isAlive())previewProcess.destroy();previewPath=path;previewProcess=ModelEditorController.javaProcess(ModelPreparationPreview.class,List.of(path.toString(),"--view",viewFile().toString()),true);
    }
    void review(String kind, boolean inspected) throws Exception {
        if(!inspected)throw new IllegalArgumentException("Inspect the collision preview and confirm the checklist before saving.");
        compile(kind);
        Path log=session.file.resolveSibling(session.file.getFileName()+"-model-check.log");lastLog=log;if(ModelEditorController.loggedProcess(ModelValidation.class,List.of(session.modelPath(kind).toString()),log).waitFor()!=0){Path proof=session.modelPath(kind).getParent().resolve("validation.json");String reason=Files.isRegularFile(proof)?MiniJson.parseObject(Files.readString(proof)).getOrDefault("error","Inspect shape coverage, masses and joints.").toString():"Inspect shape coverage, masses and joints.";throw new IllegalArgumentException("Native model check: "+reason+"\nDraft retained. Full report: "+log);}
        Path saved=Path.of(models.run("save",session.modelPath(kind).toString(),models.library.toString(),"--reviewed").toString());session.select(kind,saved);
    }
    String dimensions(String kind) throws Exception {
        var model=new ModelProfile(session.modelPath(kind),false);if(model.legacyField()){var half=FieldPackage.vector(model.parameters.get("field_half_extents_m"),2);return String.format(Locale.ROOT,"Playable field %.3f × %.3f m (%.1f × %.1f cm). Source reference drawings may extend beyond the practice layout.",half[0]*2,half[1]*2,half[0]*200,half[1]*200);}
        Path urdf=model.kind.equals("robot")?model.artifact("robot"):model.legacyField()?model.resolve("source/model.urdf"):model.artifact("field");
        var scene=new ImportedRobotScene(RobotUrdf.parse(urdf),urdf,new HardwareMap(),new DesktopAssetManager(true),8,Set.of());scene.root.updateGeometricState();
        if(!(scene.root.getWorldBound() instanceof BoundingBox b)){if(kind.equals("field")){var half=FieldPackage.vector(model.parameters.get("field_half_extents_m"),2);return String.format(Locale.ROOT,"Saved field extent %.3f × %.3f m. Open preview to inspect the generated floor and separated pieces.",half[0]*2,half[1]*2);}return "No visual bounds. Inspect the geometry and its collision preview.";}
        return String.format(Locale.ROOT,"Length %.3f m · Width %.3f m · Height %.3f m (%.1f × %.1f × %.1f cm)",b.getXExtent()*2,b.getZExtent()*2,b.getYExtent()*2,b.getXExtent()*200,b.getZExtent()*200,b.getYExtent()*200);
    }
    List<String> hardware(RobotConfigXml.DeviceType type) throws Exception {
        if(session.project()==null)return List.of();var data=MiniJson.parseObject(Files.readString(session.project().resolve("sim.config")));
        var xml=RobotConfigXml.parse(session.project().resolve(data.get("robotConfig").toString()).toFile());
        return xml.devices.stream().filter(d->type==null?Set.of(RobotConfigXml.DeviceType.MOTOR,RobotConfigXml.DeviceType.SERVO).contains(RobotConfigXml.resolveType(d.tag)):RobotConfigXml.resolveType(d.tag)==type).map(d->d.name).sorted().toList();
    }
    Map<String,Object> newScene() throws Exception {
        var out=new LinkedHashMap<String,Object>();out.put("schema_version",1);out.put("mode","field-only");out.put("piece_set","biobuzz");
        out.put("robot_start_xyz_m",List.of(-1.2,-1.2,.1));out.put("robot_start_yaw_rad",0.);out.put("pieces",new LinkedHashMap<String,Object>());return out;
    }
    Map<String,Object> sourcePieces(String pieceSet) throws Exception {
        if("generic".equals(session.reference("field")))return PracticePieces.layout(pieceSet.equals("torus"),session.profile("robot").containsKey("runtime")&&FieldPackage.map(session.profile("robot").get("runtime")).containsKey("flexible_intake"));
        var p=new ModelProfile(session.modelPath("field"),true);List<Map<String,Object>> parts=p.legacyField()?new FieldPackage(p.artifact("field")).instances.stream().filter(i->"piece".equals(i.get("owner"))).toList():FieldPackage.maps(FieldPackage.map(p.receipt.get("artifacts")).get("pieces"));
        var out=new LinkedHashMap<String,Object>();for(var part:parts){String id=FieldPackage.str(part,"id");out.put(id,new LinkedHashMap<>(Map.of("source_id",id,"label",part.getOrDefault("type",id),"enabled",true,"xyz_m",part.get("xyz_m"),"rpy_rad",part.get("rpy_rad"))));}return out;
    }
    Path saveScene(Map<String,Object> data, boolean validate) throws Exception {
        Path out=models.library.resolve("scenes/setup-"+session.file.getFileName()).toAbsolutePath();Files.createDirectories(out.getParent());out=out.getParent().toRealPath().resolve(out.getFileName());
        for(String kind:List.of("robot","field")){
            Path file=session.modelPath(kind);data.put(kind+"_profile",file==null?null:out.getParent().relativize(file.toRealPath()).toString());
            if(file!=null){var p=new ModelProfile(file,true);data.put(kind+"_identity",Map.of("profile_id",p.data.get("profile_id"),"revision_id",p.data.get("revision_id")));}else data.remove(kind+"_identity");
        }
        String proof=(String)session.data.get("scene_validation");ProfileIO.save(out,data);session.scene(out,false);
        if(proof!=null&&proof.equals(session.sceneToken())){session.data.put("scene_validation",proof);session.save();}
        if(validate){Path log=session.file.resolveSibling(session.file.getFileName()+"-scene-check.log");lastLog=log;if(ModelEditorController.loggedProcess(ScenePreflight.class,List.of(out.toString()),log).waitFor()!=0){String reason=Files.readAllLines(log).stream().filter(line->line.contains("Exception in thread")||line.startsWith("Caused by:")).findFirst().orElse("Inspect positions and shape coverage.");throw new IllegalArgumentException("Scene check failed: "+reason+"\nYour scene draft is saved. Full report: "+log);}session.scene(out,true);}return out;
    }
    void scenePreview() throws Exception {
        if(session.scenePath()==null)throw new IllegalArgumentException("Save the scene layout first.");ModelEditorController.javaProcess(GuidedScenePreview.class,List.of(session.scenePath().toString()),true);
    }
    List<String> checkProject(String policy) throws Exception {
        Path project=session.project();if(project==null)throw new IllegalArgumentException("Choose a team project containing sim.config to run code. The model can be saved without a project.");
        var config=prospectiveConfig(policy);SimConfig resolved=loadProspective(project,config);
        var xml=RobotConfigXml.parse(project.resolve(resolved.robotConfig).toFile());var preset=PresetRobotConfig.load(project.resolve(resolved.presetMotors));var hardware=HardwareMapBuilder.build(xml,preset);
        if(resolved.urdf!=null){var urdf=RobotUrdf.parse(project.resolve(resolved.urdf));urdf.validateHardwareMap(hardware);
            for(var tx:urdf.transmissions.values())for(var actuator:tx.actuators())if(hardware.tryGet(Servo.class,actuator.name())!=null&&!resolved.servoPhysics.containsKey(actuator.name()))throw new IllegalArgumentException("Servo "+actuator.name()+" needs saved effort/speed/travel settings. Bind it in Parts and movement or edit its servo physics.");}
        for(String name:resolved.drive==null?List.of("left_front_drive","right_front_drive","left_back_drive","right_back_drive"):resolved.drive.motorNames())hardware.get(simcore.SimDcMotorEx.class,name);
        if(resolved.intake!=null)hardware.get(simcore.SimDcMotorEx.class,resolved.intake.motor());
        var classes=TeamCodeCompiler.compile(project,project.resolve(resolved.sourceRoot),resolved.extraClasspath);
        try(var loader=TeamCodeCompiler.newClassLoader(project,classes,resolved.extraClasspath)){
            var choices=OpModeDiscovery.discover(classes,loader).stream().map(o->o.displayName).sorted().toList();if(choices.isEmpty())throw new IllegalArgumentException("No enabled TeleOp or Autonomous OpModes were found in this project.");
            session.data.put("project_check",GuidedSetupSession.hash(List.of(config,hardware(RobotConfigXml.DeviceType.MOTOR),hardware(RobotConfigXml.DeviceType.SERVO))));session.data.put("opmodes",choices);session.data.put("model_settings",policy);session.save();return choices;
        }finally{try(var paths=Files.walk(classes)){for(Path p:paths.sorted(Comparator.reverseOrder()).toList())Files.deleteIfExists(p);}}
    }
    private Map<String,Object> prospectiveConfig(String policy) throws Exception {
        var root=MiniJson.parseObject(Files.readString(session.project().resolve("sim.config")));root.put("model_settings",policy);
        if(session.goal().equals("simulation")){if(!session.sceneReady())throw new IllegalArgumentException("Validate the scene before using it in a project.");root.put("scene_profile",session.scenePath().toString());}
        else if(session.goal().equals("robot")){root.put("robot_model_profile",session.modelPath("robot").toString());root.remove("scene_profile");}
        else {root.put("field",Map.of("source","imported","package",session.modelPath("field").toString(),"mode","field-only"));root.remove("scene_profile");}
        return root;
    }
    private SimConfig loadProspective(Path project,Map<String,Object> root) throws Exception {return SimConfig.parse(project,root);}
    String provisionalSummary(String kind) throws Exception {
        var p=session.profile(kind);var params=FieldPackage.map(p.get("parameters"));int masses=0,boxes=0;
        for(var value:FieldPackage.map(p.get("entities")).values()){var settings=FieldPackage.map(FieldPackage.map(value).get("settings"));if("fallback".equals(settings.get("mass_mode")))masses++;if("box".equals(settings.get("collision_strategy")))boxes++;}
        return masses+" parts use fallback mass; "+boxes+" use box proxies. Shared friction "+params.get("friction")+", collision margin "+params.get("collision_margin_m")+" m. Check model provenance before treating these as measured.";
    }
    void applyProject(String policy) throws Exception {
        checkProject(policy);
        if(session.goal().equals("simulation"))ProfileIO.save(session.project().resolve("sim.config"),prospectiveConfig(policy));
        else ProjectModelSelection.apply(session.project(),new ModelProfile(session.modelPath(session.goal()),true),policy);
    }
}
