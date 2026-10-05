package simrunner;

import java.nio.file.*;
import java.util.*;
import simcore.MiniJson;

/** Compare prepared CAD poses to an explicit reference. Count checks do not certify a match setup. */
final class FieldLayoutAudit {
    static void validate(Map<String,Object> reference){
        if(!Set.of("source_url","revision","tolerance_m","instances").containsAll(reference.keySet()))throw new IllegalArgumentException("Unknown layout reference setting");
        String source=FieldPackage.str(reference,"source_url");if(!source.startsWith("https://"))throw new IllegalArgumentException("Layout reference requires HTTPS source URL");FieldPackage.str(reference,"revision");SceneSensorConfig.n(reference,"tolerance_m",.005,.00001,.1);
        var ids=new HashSet<String>();for(var e:FieldPackage.maps(reference.get("instances"))){if(!ids.add(FieldPackage.str(e,"id")))throw new IllegalArgumentException("Duplicate layout reference ID");FieldPackage.pos(e.get("xyz_m"));}if(ids.isEmpty())throw new IllegalArgumentException("Layout reference needs at least one pose");
    }
    static Map<String,Object> audit(Map<String,Object> field,Map<String,Object> reference) {
        var checks=new ArrayList<Map<String,Object>>();var instances=FieldPackage.maps(field.get("instances"));var meshes=FieldPackage.map(field.get("meshes"));
        count(checks,"hives",FieldPackage.maps(field.get("hives")).size(),2);
        count(checks,"flowers",instances.stream().filter(i->FieldPackage.str(i,"mesh").startsWith("am_5857__Flower_Layer_C")).count(),4);
        count(checks,"pollen inventory",instances.stream().filter(i->"pollen".equals(i.get("category"))).count(),40);
        count(checks,"red nectar inventory",instances.stream().filter(i->"red_nectar".equals(i.get("category"))).count(),8);
        count(checks,"blue nectar inventory",instances.stream().filter(i->"blue_nectar".equals(i.get("category"))).count(),8);
        if(!reference.isEmpty()){validate(reference);double tolerance=SceneSensorConfig.n(reference,"tolerance_m",.005,.00001,.1);
            for(var e:FieldPackage.maps(reference.get("instances"))){String id=FieldPackage.str(e,"id");var actual=instances.stream().filter(i->id.equals(i.get("id"))).findFirst();double error=actual.isEmpty()?Double.POSITIVE_INFINITY:FieldPackage.pos(actual.get().get("xyz_m")).distance(FieldPackage.pos(e.get("xyz_m")));checks.add(Map.of("check","pose "+id,"pass",error<=tolerance,"error_m",Double.isFinite(error)?error:"missing","tolerance_m",tolerance));}}
        var uncertainties=new ArrayList<String>();uncertainties.add("Inventory counts are assembly inventory, not a verified initial placement or robot preload.");uncertainties.add("Tape boundaries, tag poses, hive facing, piece distribution and damper calibration require setup verification.");
        if(reference.isEmpty())uncertainties.add("No reference poses supplied; all CAD placements remain unverified.");else uncertainties.add("Reference checks cover only listed centers; orientations, other parts and physical setup remain unverified.");
        for(var a:(List<?>)field.getOrDefault("approximations",List.of()))uncertainties.add(a.toString());
        return Map.of("schema_version",1,"competition_layout_verified",false,"source_url","https://ftc-resources.firstinspires.org/ftc/field/eventfieldguide","source_revision","V1.0 2026-09-12","reference",reference,"checks",checks,"uncertainties",uncertainties,"prepared_meshes",meshes.size());
    }
    private static void count(List<Map<String,Object>> checks,String id,long actual,int expected){checks.add(Map.of("check",id,"actual",actual,"expected",expected,"pass",actual==expected));}
    public static void main(String[] args)throws Exception {
        if(args.length<2||args.length>3)throw new IllegalArgumentException("FieldLayoutAudit field.json report.json [reference.json]");
        var field=MiniJson.parseObject(Files.readString(Path.of(args[0])));var reference=args.length==3?MiniJson.parseObject(Files.readString(Path.of(args[2]))):Map.<String,Object>of();var report=audit(field,reference);
        Path out=Path.of(args[1]).toAbsolutePath();Files.createDirectories(out.getParent());Files.writeString(out,ProfileIO.json(report));System.out.println("Layout audit saved: "+out+" | competition placement remains unverified");
    }
}
