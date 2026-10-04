package simrunner;

import physics.calibration.GripCalibrator;
import physics.calibration.GripCalibrator.*;
import java.nio.file.*;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.util.*;

/** Unit conversion, CSV exchange and self-contained, rechecked wheel measurement evidence. */
final class WheelGripMeasurement {
    enum LoadUnit {
        N("N",1,"normal_load_n"),KG("kg",9.80665,"normal_mass_kg"),LB("lb",.45359237*9.80665,"normal_mass_lb");
        final double newtons;final String column,label;LoadUnit(String label,double factor,String column){this.label=label;newtons=factor;this.column=column;}public String toString(){return label;}
    }
    enum ForceUnit {
        N("N",1,"sliding_force_n"),LBF("lbf",4.4482216152605,"sliding_force_lbf");
        final double newtons;final String column,label;ForceUnit(String label,double factor,String column){this.label=label;newtons=factor;this.column=column;}public String toString(){return label;}
    }
    record Dataset(List<Reading> readings,int otherWheelRows) {Dataset{readings=List.copyOf(readings);}}
    record Checked(String surface,double surfaceCoefficient,Criteria criteria,List<Reading> readings,Result result) {Checked{readings=List.copyOf(readings);}}
    static Dataset readCsv(Path file,String joint)throws Exception {
        if(Files.size(file)>1_000_000)throw new IllegalArgumentException("Grip CSV exceeds 1 MB.");
        var lines=Files.readAllLines(file);if(lines.isEmpty())throw new IllegalArgumentException("Grip CSV is empty.");
        var header=csvLine(lines.get(0).replaceFirst("^\\uFEFF",""));var columns=new LinkedHashMap<String,Integer>();for(int i=0;i<header.size();i++)if(columns.putIfAbsent(header.get(i).strip(),i)!=null)throw new IllegalArgumentException("Duplicate grip CSV column: "+header.get(i));
        LoadUnit load=null;ForceUnit force=null;
        for(var unit:LoadUnit.values())if(columns.containsKey(unit.column)){if(load!=null)throw new IllegalArgumentException("Choose exactly one normal-load unit column.");load=unit;}
        for(var unit:ForceUnit.values())if(columns.containsKey(unit.column)){if(force!=null)throw new IllegalArgumentException("Choose exactly one force unit column.");force=unit;}
        if(!columns.keySet().containsAll(List.of("joint","trial","set"))||columns.size()!=5||load==null||force==null)throw new IllegalArgumentException("Use joint,trial,set; one normal_load_n/normal_mass_kg/normal_mass_lb column; and one sliding_force_n/sliding_force_lbf column.");
        var readings=new ArrayList<Reading>();var sets=new HashMap<String,String>();int other=0,total=0;
        for(int i=1;i<lines.size();i++) {
            if(lines.get(i).isBlank())continue;
            try {
                if(++total>1000)throw new IllegalArgumentException("At most 1000 readings per CSV.");
                var row=csvLine(lines.get(i));if(row.size()!=header.size())throw new IllegalArgumentException("Wrong column count.");
                String name=row.get(columns.get("joint")).strip();if(name.isBlank())throw new IllegalArgumentException("Missing wheel joint.");
                var r=new Reading(row.get(columns.get("trial")).strip(),row.get(columns.get("set")).strip(),Double.parseDouble(row.get(columns.get(load.column)).strip())*load.newtons,Double.parseDouble(row.get(columns.get(force.column)).strip())*force.newtons);
                if(sets.putIfAbsent(r.trial(),r.set())!=null&&!sets.get(r.trial()).equals(r.set()))throw new IllegalArgumentException("A physical trial appears in both sets; reserve whole trials for validation.");
                if(joint.equals(name))readings.add(r);else other++;
            }catch(RuntimeException e){throw new IllegalArgumentException("Grip CSV line "+(i+1)+": "+e.getMessage(),e);}
        }
        if(readings.isEmpty())throw new IllegalArgumentException("No readings for selected joint "+joint+". Choose the recorded wheel or correct its CSV joint labels explicitly.");
        return new Dataset(readings,other);
    }
    static void template(Path file,String joint,LoadUnit load,ForceUnit force)throws Exception {
        var csv=new StringBuilder("joint,trial,set,"+load.column+","+force.column+"\n");
        for(int i=0;i<5;i++)csv.append(quoted(joint)).append(',').append(i<3?"fit-"+(i+1):"check-"+(i-2)).append(',').append(i<3?"fit":"validate").append(",,\n");
        Files.writeString(file,csv.toString());
    }
    static String normalizedCsv(String joint,List<Reading> readings) {
        var csv=new StringBuilder("joint,trial,set,normal_load_n,sliding_force_n\n");
        for(var r:readings)csv.append(quoted(joint)).append(',').append(quoted(r.trial())).append(',').append(r.set()).append(',').append(r.normalN()).append(',').append(r.forceN()).append('\n');
        return csv.toString();
    }
    static Map<String,Object> evidence(String joint,String surface,double surfaceCoefficient,Criteria criteria,List<Reading> readings)throws Exception {
        if(surface==null||surface.isBlank()||surface.length()>200)throw new IllegalArgumentException("Name the actual reference surface (at most 200 characters).");
        var result=GripCalibrator.fit(readings,surfaceCoefficient,criteria);if(!result.accepted())throw new IllegalArgumentException(String.join("\n",result.issues()));
        String recordingHash=hash(normalizedCsv(joint,readings));
        return new LinkedHashMap<>(Map.of("schema_version",1,"method","restrained-wheel sliding onset","recorded_joint",joint,"reference_surface",surface.strip(),"surface_friction",surfaceCoefficient,"criteria",criteria(criteria),"readings",readings.stream().map(r->Map.of("trial",r.trial(),"set",r.set(),"normal_load_n",r.normalN(),"sliding_force_n",r.forceN())).toList(),"recording_sha256",recordingHash,"fit_context_sha256",contextHash(recordingHash,surface.strip(),surfaceCoefficient,criteria),"result",metrics(result)));
    }
    static Checked check(Map<String,Object> evidence,double coefficient)throws Exception {
        if(FieldPackage.num(evidence,"schema_version")!=1||!"restrained-wheel sliding onset".equals(evidence.get("method")))throw new IllegalArgumentException("Unsupported wheel measurement format.");
        String joint=FieldPackage.str(evidence,"recorded_joint"),surface=FieldPackage.str(evidence,"reference_surface");if(surface.isBlank()||surface.length()>200||joint.isBlank())throw new IllegalArgumentException("Saved wheel measurements need their original joint and surface name.");
        var criteria=criteria(FieldPackage.map(evidence.get("criteria")));var readings=readings(evidence);
        if(!hash(normalizedCsv(joint,readings)).equals(evidence.get("recording_sha256")))throw new IllegalArgumentException("Saved grip readings changed. Refit the measurements before applying them.");
        double surfaceCoefficient=FieldPackage.num(evidence,"surface_friction");
        if(!contextHash(evidence.get("recording_sha256").toString(),surface,surfaceCoefficient,criteria).equals(evidence.get("fit_context_sha256")))throw new IllegalArgumentException("Saved grip surface or quality criteria changed. Refit the measurements.");
        var result=GripCalibrator.fit(readings,surfaceCoefficient,criteria);
        if(!result.accepted()||Math.abs(result.wheelCoefficient()-coefficient)>1e-9)throw new IllegalArgumentException("Wheel coefficient no longer matches passing measurement evidence. Refit it or discard the saved evidence to keep a manual assumption.");
        var expected=metrics(result);var stored=FieldPackage.map(evidence.get("result"));
        for(var entry:expected.entrySet()){double a=((Number)entry.getValue()).doubleValue(),b=FieldPackage.num(stored,entry.getKey());if(Math.abs(a-b)>1e-9*Math.max(1,Math.abs(a)))throw new IllegalArgumentException("Saved grip fit results changed. Refit the measurements.");}
        return new Checked(surface,surfaceCoefficient,criteria,readings,result);
    }
    static List<Reading> readings(Map<String,Object> evidence) {
        var rows=FieldPackage.maps(evidence.get("readings"));if(rows.size()>1000)throw new IllegalArgumentException("Saved grip evidence exceeds 1000 readings.");
        return rows.stream().map(r->new Reading(FieldPackage.str(r,"trial"),FieldPackage.str(r,"set"),FieldPackage.num(r,"normal_load_n"),FieldPackage.num(r,"sliding_force_n"))).toList();
    }
    static Map<String,Object> criteria(Criteria c){return Map.of("min_fit_trials",c.minFitTrials(),"min_validation_trials",c.minValidationTrials(),"min_load_variation",c.minLoadVariation(),"max_relative_rmse",c.maxRelativeRmse(),"max_trial_relative_rmse",c.maxTrialRelativeRmse());}
    static Criteria criteria(Map<String,Object> c){double fit=FieldPackage.num(c,"min_fit_trials"),validation=FieldPackage.num(c,"min_validation_trials");if(fit!=(int)fit||validation!=(int)validation)throw new IllegalArgumentException("Grip trial counts must be integers.");return new Criteria((int)fit,(int)validation,FieldPackage.num(c,"min_load_variation"),FieldPackage.num(c,"max_relative_rmse"),FieldPackage.num(c,"max_trial_relative_rmse"));}
    static Map<String,Object> metrics(Result r){return Map.of("effective_mu",r.effectiveMu(),"wheel_coefficient",r.wheelCoefficient(),"fit_trials",r.fitTrials(),"validation_trials",r.validationTrials(),"load_variation",r.loadVariation(),"fit_rmse_n",r.fitRmseN(),"validation_rmse_n",r.validationRmseN(),"fit_relative_rmse",r.fitRelativeRmse(),"validation_relative_rmse",r.validationRelativeRmse(),"worst_trial_relative_rmse",r.worstTrialRelativeRmse());}
    private static String hash(String value)throws Exception{return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(value.getBytes(StandardCharsets.UTF_8)));}
    private static String contextHash(String recordingHash,String surface,double coefficient,Criteria c)throws Exception{return hash(recordingHash+"\n"+ProfileIO.json(surface)+"\n"+coefficient+"\n"+c.minFitTrials()+","+c.minValidationTrials()+","+c.minLoadVariation()+","+c.maxRelativeRmse()+","+c.maxTrialRelativeRmse());}
    private static String quoted(String value){if(value.contains("\n")||value.contains("\r"))throw new IllegalArgumentException("CSV names must fit on one line.");return "\""+value.replace("\"","\"\"")+"\"";}
    private static List<String> csvLine(String line) {
        var values=new ArrayList<String>();var value=new StringBuilder();boolean quote=false,closed=false;
        for(int i=0;i<line.length();i++){char c=line.charAt(i);if(quote){if(c=='"'){if(i+1<line.length()&&line.charAt(i+1)=='"'){value.append('"');i++;}else {quote=false;closed=true;}}else value.append(c);}else if(c==','){values.add(value.toString());value.setLength(0);closed=false;}else if(c=='"'){if(value.length()!=0||closed)throw new IllegalArgumentException("Unexpected CSV quote.");quote=true;}else if(closed){if(!Character.isWhitespace(c))throw new IllegalArgumentException("Unexpected text after quoted CSV value.");}else value.append(c);}
        if(quote)throw new IllegalArgumentException("Unclosed CSV quote; multiline values are unsupported.");values.add(value.toString());return values;
    }
}
