package simrunner;

import java.time.Instant;
import java.util.*;

/** User assessments of exact native observations, independent of physical/collision approval. */
final class RobotMotionReview {
    static final List<String> ASSESSMENTS=List.of("Not reviewed","Correct","Reversed","Blocked","Incorrect travel");
    enum Correction {
        BINDING("Motor binding and signed gearing","parts"),
        JOINT("Joint axis and travel limits","parts"),
        COLLISION("Collision geometry","physics"),
        EFFORT("Motor effort and servo travel","physics"),
        DRIVE("Drivetrain shaft signs and dimensions","physics");
        final String label,page;Correction(String label,String page){this.label=label;this.page=page;}
        public String toString(){return label;}
    }
    record Row(RobotMotionPlan.Item item,Map<String,Object> observation,String runId,
               String assessment,String state,boolean reviewable,String note) { }
    record Snapshot(RobotMotionDemo.Setup setup,Map<String,Object> report,List<Row> rows,String summary) { }

    static Map<String,Object> object(Object value){return value instanceof Map?FieldPackage.map(value):Map.of();}
    static boolean current(RobotMotionDemo.Setup setup,Map<String,Object> data){return Objects.equals(data.get("model_digest"),setup.profile().digest)&&Objects.equals(data.get("context"),setup.context());}
    static boolean observable(Map<String,Object> row){return row.containsKey("forward_m")||row.containsKey("start");}
    static Map<String,Object> entries(Map<String,Object> profile){return object(object(profile.get("motion_review")).get("entries"));}
    static Snapshot read(RobotMotionDemo.Setup setup,Map<String,Object> report) {
        var observations=new HashMap<String,Map<String,Object>>();
        for(var o:FieldPackage.maps(report.getOrDefault("observations",List.of())))if(o.get("id") instanceof String id)observations.put(id,o);
        boolean fresh=current(setup,report)&&report.get("schema_version") instanceof Number version&&version.intValue()==2;
        boolean complete=fresh&&Boolean.TRUE.equals(report.get("complete"))&&"finished".equals(report.get("status"));
        String latestRun=Objects.toString(report.get("run_id"),"");
        var saved=entries(setup.profile().data);var rows=new ArrayList<Row>();
        for(var item:setup.plan().items()) {
            var old=object(saved.get(item.id()));boolean valid=current(setup,old)&&!Boolean.TRUE.equals(old.get("retest_required"));
            boolean covered=fresh&&(Objects.toString(report.get("selection"),"").isEmpty()||Objects.equals(item.group(),report.get("selection")));
            var observed=covered?observations.getOrDefault(item.id(),Map.of()):Map.<String,Object>of();
            String run=covered?latestRun:Objects.toString(old.get("run_id"),"");
            boolean reviewed=valid&&(!covered||Objects.equals(run,old.get("run_id")))&&ASSESSMENTS.contains(old.get("assessment"));
            String state=reviewed?"Reviewed":covered?complete&&observable(observed)?"New observation — review needed":"Retest not finished":old.isEmpty()?"Run demo first":"CAD/settings/hardware changed or retest requested";
            if(!reviewed&&!old.isEmpty())state+="; previously "+old.get("assessment");
            if(!covered)observed=object(old.get("observation"));
            boolean reviewable=covered?complete&&observable(observed)&&!run.isEmpty():valid&&observable(observed)&&!run.isEmpty();
            rows.add(new Row(item,observed,run,reviewed?old.get("assessment").toString():"Not reviewed",state,reviewable,Objects.toString(old.get("note"),"")));
        }
        String message=setup.hardwareLabel()+"\nReview each observed movement against your robot. Retests require another explicit assessment; collision review is separate.";
        if(report.containsKey("error"))message+="\nLast demo error: "+report.get("error");
        else if(!report.isEmpty()&&!fresh)message+="\nLast report belongs to earlier settings/hardware or an older demo. Retest current movements.";
        if(rows.isEmpty())message+="\nNo configured movements. "+setup.plan().summary();
        return new Snapshot(setup,report,List.copyOf(rows),message);
    }
    static Map<String,Object> assess(Snapshot snapshot,Map<String,String> choices,Map<String,String> notes) {
        var result=new LinkedHashMap<String,Object>(object(snapshot.setup().profile().data.get("motion_review")));
        var entries=new LinkedHashMap<String,Object>(entries(snapshot.setup().profile().data));
        for(var choice:choices.entrySet()) {
            if(!ASSESSMENTS.contains(choice.getValue())||choice.getValue().equals("Not reviewed"))throw new IllegalArgumentException("Choose Correct, Reversed, Blocked or Incorrect travel.");
            Row row=snapshot.rows().stream().filter(r->r.item().id().equals(choice.getKey())).findFirst().orElseThrow(()->new IllegalArgumentException("Movement is no longer configured."));
            if(!row.reviewable())throw new IllegalArgumentException("Retest and finish this movement before saving its assessment: "+row.item().label());
            if(choice.getValue().equals("Correct")&&!"movement observed".equals(row.observation().get("outcome")))throw new IllegalArgumentException("The native observation did not confirm movement. Correct the setup and retest before marking it Correct.");
            String note=notes.getOrDefault(choice.getKey(),row.note());if(note.length()>2000)throw new IllegalArgumentException("Keep each movement note under 2000 characters.");
            var entry=new LinkedHashMap<String,Object>();entry.put("assessment",choice.getValue());entry.put("note",note);entry.put("group",row.item().group());entry.put("model_digest",snapshot.setup().profile().digest);entry.put("source_fingerprint",object(snapshot.setup().profile().data.get("source")).get("fingerprint"));entry.put("context",snapshot.setup().context());entry.put("hardware",snapshot.setup().hardwareLabel());entry.put("run_id",row.runId());entry.put("observed_revision_id",snapshot.setup().profile().data.get("revision_id"));entry.put("observation",row.observation());entry.put("reviewed_at",Instant.now().toString());entry.put("retest_required",false);entries.put(choice.getKey(),entry);
        }
        result.put("schema_version",1);result.put("entries",entries);return result;
    }
    static Map<String,Object> requireRetest(Map<String,Object> profile,String group) {
        var result=new LinkedHashMap<String,Object>(object(profile.get("motion_review")));var entries=new LinkedHashMap<String,Object>();
        entries(profile).forEach((id,value)->{var entry=new LinkedHashMap<String,Object>(object(value));if(group.isEmpty()||group.equals(entry.get("group")))entry.put("retest_required",true);entries.put(id,entry);});
        result.put("schema_version",1);result.put("entries",entries);return result;
    }
    static String travel(Map<String,Object> o){
        if(o.containsKey("forward_m"))return String.format(Locale.ROOT,"Forward %.3f m · left %.3f m · turn %.1f°",FieldPackage.num(o,"forward_m"),FieldPackage.num(o,"left_m"),Math.toDegrees(FieldPackage.num(o,"yaw_rad")));
        if(o.containsKey("start"))return String.format(Locale.ROOT,"%.3f → %.3f %s; target %.3f %s",FieldPackage.num(o,"start"),FieldPackage.num(o,"end"),o.get("unit"),FieldPackage.num(o,"target"),o.get("unit"));
        return "No completed observation";
    }
    static String advice(Row row,Correction correction){
        String target=row.item().part().isEmpty()?"the driven wheel parts":row.item().part();
        return switch(correction){
            case BINDING->"Check "+target+": bind the intended motor/servo and verify the signed motor-to-joint ratio. A matching hardware name does not verify wiring. Mechanism demos control joint targets and compensate signed gearing; check joint axes for visible reversal, and gearing for TeamCode/encoder semantics.";
            case JOINT->"Check "+target+": joint movement, axis direction and lower/upper limits. Hinge limits use radians; sliders use the selected source length units. The demo uses nearby targets, not full travel.";
            case COLLISION->"Inspect "+target+" in the collision preview. Check wheel/floor alignment, openings and contact shapes that could block travel. Directly joined bodies are collision-exempt; other bodies can contact.";
            case EFFORT->"Check Runtime / calibrated settings: saved servo torque, speed and travel. Motors use selected project presets (or disclosed generic demo motors); check those presets and physical load before changing limits. Do not infer measured effort from CAD.";
            case DRIVE->"Check Runtime / calibrated settings: differential left/right shaft signs, motor names and measured wheel/track dimensions. Mecanum uses four named wheel slots; confirm each binding and its axis in Parts and movement. Changing shaft signs can also affect turning.";
        };
    }
    static Correction recommended(Row row){return switch(row.assessment()){case "Reversed"->row.item().part().isEmpty()?Correction.DRIVE:Correction.JOINT;case "Blocked"->Correction.COLLISION;case "Incorrect travel"->row.item().part().isEmpty()?Correction.DRIVE:Correction.JOINT;default->Correction.BINDING;};}
}
