package simrunner;

import physics.TireFriction;
import physics.calibration.TireSlipCalibrator;
import physics.calibration.TireSlipCalibrator.*;
import java.nio.file.*;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.util.*;

/** Portable direct-force tire recordings. SI units; no current-derived force inference. */
final class TireSlipMeasurement {
    record Row(String recordedJoint,Reading reading) {
        Row {if(recordedJoint==null||recordedJoint.isBlank()||recordedJoint.contains("\n")||recordedJoint.contains("\r"))throw new IllegalArgumentException("Name the recorded wheel joint");}
    }
    record Context(String tireClass,String surface,String hubSource,String forceSource) {
        Context {if(!List.of("traction","omni").contains(tireClass))throw new IllegalArgumentException("Choose traction or omni tire class");for(String text:List.of(surface,hubSource,forceSource))if(text.isBlank()||text.length()>200)throw new IllegalArgumentException("Name the reference surface, independent hub-speed measurement and direct force measurement (≤200 characters each)");}
    }
    record Checked(Context context,Bounds bounds,Criteria criteria,List<Row> rows,Result result) {Checked{rows=List.copyOf(rows);}}
    static List<Row> readCsv(Path file,Set<String> joints)throws Exception {
        if(Files.size(file)>1_000_000)throw new IllegalArgumentException("Tire CSV exceeds 1 MB");var lines=Files.readAllLines(file);if(lines.isEmpty())throw new IllegalArgumentException("Empty tire CSV");
        var header=WheelGripMeasurement.csvLine(lines.get(0).replaceFirst("^\\uFEFF",""));var columns=new HashMap<String,Integer>();for(int i=0;i<header.size();i++)if(columns.putIfAbsent(header.get(i).strip(),i)!=null)throw new IllegalArgumentException("Duplicate tire CSV column");
        var required=List.of("joint","trial","set","wheel_surface_mps","hub_speed_mps","normal_load_n","longitudinal_force_n");if(columns.size()!=7||!columns.keySet().containsAll(required))throw new IllegalArgumentException("Use SI headers: "+String.join(",",required));
        var rows=new ArrayList<Row>();var partitions=new HashMap<String,String>();var owners=new HashMap<String,String>();int total=0;
        for(int i=1;i<lines.size();i++){if(lines.get(i).isBlank())continue;try{
            if(++total>1000)throw new IllegalArgumentException("At most 1000 readings");var fields=WheelGripMeasurement.csvLine(lines.get(i));if(fields.size()!=7)throw new IllegalArgumentException("Wrong column count");
            String joint=fields.get(columns.get("joint")).strip(),id=fields.get(columns.get("trial")).strip(),set=fields.get(columns.get("set")).strip();
            var r=new Row(joint,new Reading(id,set,number(fields,columns,"wheel_surface_mps"),number(fields,columns,"hub_speed_mps"),number(fields,columns,"normal_load_n"),number(fields,columns,"longitudinal_force_n")));
            if(partitions.putIfAbsent(id,set)!=null&&!partitions.get(id).equals(set))throw new IllegalArgumentException("Whole trial appears in both sets");if(owners.putIfAbsent(id,joint)!=null&&!owners.get(id).equals(joint))throw new IllegalArgumentException("Use distinct trial IDs for different wheel experiments");
            if(!joints.contains(joint))throw new IllegalArgumentException("Joint "+joint+" is outside the selected tire class. Choose a listed joint or the correct class explicitly");rows.add(r);
        }catch(RuntimeException e){throw new IllegalArgumentException("Tire CSV line "+(i+1)+": "+e.getMessage(),e);}}
        if(rows.isEmpty())throw new IllegalArgumentException("No tire readings");return List.copyOf(rows);
    }
    private static double number(List<String> row,Map<String,Integer> columns,String key){return Double.parseDouble(row.get(columns.get(key)).strip());}
    static String csv(List<Row> rows){var out=new StringBuilder("joint,trial,set,wheel_surface_mps,hub_speed_mps,normal_load_n,longitudinal_force_n\n");for(var row:rows){var r=row.reading;out.append(quote(row.recordedJoint)).append(',').append(quote(r.trial())).append(',').append(r.set()).append(',').append(r.wheelMps()).append(',').append(r.hubMps()).append(',').append(r.normalN()).append(',').append(r.forceN()).append('\n');}return out.toString();}
    static void template(Path file,String joint)throws Exception {var out=new StringBuilder(csv(List.of()));for(int i=0;i<18;i++)out.append(quote(joint)).append(',').append(i<12?"fit-"+(i+1):"check-"+(i-11)).append(',').append(i<12?"fit":"validate").append(",,,,\n");Files.writeString(file,out.toString());}
    private static String quote(String v){return "\""+v.replace("\"","\"\"")+"\"";}
    static Result fit(List<Row> rows,double lateral,Bounds bounds,Criteria criteria){
        var owners=new HashMap<String,String>();for(var row:rows)if(owners.putIfAbsent(row.reading.trial(),row.recordedJoint)!=null&&!owners.get(row.reading.trial()).equals(row.recordedJoint))throw new IllegalArgumentException("A trial contains different wheel experiments");
        return TireSlipCalibrator.fit(rows.stream().map(Row::reading).toList(),lateral,bounds,criteria);
    }
    static Map<String,Object> evidence(Context context,List<Row> rows,double lateral,Bounds bounds,Criteria criteria)throws Exception {
        var result=fit(rows,lateral,bounds,criteria);if(!result.accepted())throw new IllegalArgumentException(String.join("\n",result.issues()));
        var e=new LinkedHashMap<String,Object>();e.put("schema_version",1);e.put("method","steady direct-force longitudinal brush");e.put("tire_class",context.tireClass);e.put("reference_surface",context.surface);e.put("hub_speed_source",context.hubSource);e.put("force_source",context.forceSource);e.put("bounds",bounds(bounds));e.put("criteria",criteria(criteria));
        e.put("readings",rows.stream().map(row->{var r=row.reading;return Map.of("recorded_joint",row.recordedJoint,"trial",r.trial(),"set",r.set(),"wheel_surface_mps",r.wheelMps(),"hub_speed_mps",r.hubMps(),"normal_load_n",r.normalN(),"longitudinal_force_n",r.forceN());}).toList());
        e.put("context_sha256",hash(e));e.put("result",metrics(result));return e;
    }
    static Checked check(Map<String,Object> e,String tireClass,TireFriction.Spec spec)throws Exception {
        if(FieldPackage.num(e,"schema_version")!=1||!"steady direct-force longitudinal brush".equals(e.get("method")))throw new IllegalArgumentException("Unsupported tire measurement evidence");
        var context=context(e);if(!context.tireClass.equals(tireClass))throw new IllegalArgumentException("Tire evidence belongs to a different class");
        var content=new LinkedHashMap<>(e);content.remove("context_sha256");content.remove("result");if(!hash(content).equals(e.get("context_sha256")))throw new IllegalArgumentException("Tire readings/context changed; refit or discard evidence");
        var bounds=readBounds(FieldPackage.map(e.get("bounds")));var criteria=readCriteria(FieldPackage.map(e.get("criteria")));var rows=readings(e);var r=fit(rows,spec.lateralScale(),bounds,criteria);
        if(!r.accepted())throw new IllegalArgumentException("Tire evidence no longer passes quality checks: "+String.join("; ",r.issues()));
        for(String key:List.of("static_mu","sliding_mu","stiffness_n_per_mps","transition_mps"))if(Math.abs(FieldPackage.num(spec(r.spec()),key)-FieldPackage.num(spec(spec),key))>1e-7*Math.max(1,FieldPackage.num(spec(spec),key)))throw new IllegalArgumentException("Tire "+key+" no longer matches measurements; refit or discard evidence");
        var stored=FieldPackage.map(e.get("result"));for(var entry:metrics(r).entrySet()){double a=((Number)entry.getValue()).doubleValue(),b=FieldPackage.num(stored,entry.getKey());if(Math.abs(a-b)>1e-7*Math.max(1,Math.abs(a)))throw new IllegalArgumentException("Tire fit results changed; refit evidence");}
        return new Checked(context,bounds,criteria,rows,r);
    }
    static Context context(Map<String,Object> e){return new Context(FieldPackage.str(e,"tire_class"),FieldPackage.str(e,"reference_surface"),FieldPackage.str(e,"hub_speed_source"),FieldPackage.str(e,"force_source"));}
    static List<Row> readings(Map<String,Object> e){var values=FieldPackage.maps(e.get("readings"));if(values.size()>1000)throw new IllegalArgumentException("Too many tire readings");return values.stream().map(v->new Row(FieldPackage.str(v,"recorded_joint"),new Reading(FieldPackage.str(v,"trial"),FieldPackage.str(v,"set"),FieldPackage.num(v,"wheel_surface_mps"),FieldPackage.num(v,"hub_speed_mps"),FieldPackage.num(v,"normal_load_n"),FieldPackage.num(v,"longitudinal_force_n")))).toList();}
    static Map<String,Object> spec(TireFriction.Spec s){return new LinkedHashMap<>(Map.of("static_mu",s.staticMu(),"sliding_mu",s.slidingMu(),"stiffness_n_per_mps",s.stiffnessNPerMps(),"transition_mps",s.transitionMps(),"lateral_scale",s.lateralScale()));}
    static Map<String,Object> bounds(Bounds b){return Map.of("min_static",b.minStatic(),"max_static",b.maxStatic(),"min_sliding",b.minSliding(),"max_sliding",b.maxSliding(),"min_stiffness",b.minStiffness(),"max_stiffness",b.maxStiffness(),"min_transition",b.minTransition(),"max_transition",b.maxTransition());}
    static Bounds readBounds(Map<String,Object> m){return new Bounds(n(m,"min_static"),n(m,"max_static"),n(m,"min_sliding"),n(m,"max_sliding"),n(m,"min_stiffness"),n(m,"max_stiffness"),n(m,"min_transition"),n(m,"max_transition"));}
    static Map<String,Object> criteria(Criteria c){return Map.of("min_fit_trials",c.minFitTrials(),"min_validation_trials",c.minValidationTrials(),"min_load_variation",c.minLoadVariation(),"max_relative_rmse",c.maxRelativeRmse(),"max_trial_error",c.maxTrialError(),"max_slip_spread",c.maxSlipSpread(),"max_load_spread",c.maxLoadSpread(),"min_sensitivity",c.minSensitivity(),"min_information_eigenvalue",c.minInformationEigenvalue());}
    static Criteria readCriteria(Map<String,Object> m){double f=n(m,"min_fit_trials"),v=n(m,"min_validation_trials");if(f!=(int)f||v!=(int)v)throw new IllegalArgumentException("Trial counts must be integers");return new Criteria((int)f,(int)v,n(m,"min_load_variation"),n(m,"max_relative_rmse"),n(m,"max_trial_error"),n(m,"max_slip_spread"),n(m,"max_load_spread"),n(m,"min_sensitivity"),n(m,"min_information_eigenvalue"));}
    private static double n(Map<String,Object> m,String k){return FieldPackage.num(m,k);}
    static Map<String,Object> metrics(Result r){var m=spec(r.spec());m.remove("lateral_scale");m.putAll(Map.of("fit_trials",r.fitTrials(),"validation_trials",r.validationTrials(),"load_variation",r.loadVariation(),"fit_rmse_n",r.fitRmseN(),"validation_rmse_n",r.validationRmseN(),"fit_relative_rmse",r.fitRelativeRmse(),"validation_relative_rmse",r.validationRelativeRmse(),"worst_trial_error",r.worstTrialError(),"min_sensitivity",r.minSensitivity(),"information_eigenvalue",r.informationEigenvalue()));return m;}
    /** Sort keys and normalize numbers so JSON integer/decimal representation does not affect identity. */
    private static Object canonical(Object v){if(v instanceof Map<?,?> m){var sorted=new TreeMap<String,Object>();m.forEach((k,x)->sorted.put(k.toString(),canonical(x)));return sorted;}if(v instanceof List<?> l)return l.stream().map(TireSlipMeasurement::canonical).toList();if(v instanceof Number n)return n.doubleValue();return v;}
    private static String hash(Object value)throws Exception{return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(ProfileIO.json(canonical(value)).getBytes(StandardCharsets.UTF_8)));}
}
