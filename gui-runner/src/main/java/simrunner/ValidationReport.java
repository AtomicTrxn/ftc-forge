package simrunner;

import java.nio.file.*;
import java.time.Instant;
import java.util.*;

/** Aggregates independent scenarios so a failure cannot masquerade as a successful partial run. */
final class ValidationReport {
    @FunctionalInterface interface Scenario { Map<String,Object> run() throws Exception; }
    final List<Map<String,Object>> cases=new ArrayList<>();
    final Map<String,Object> metadata=new LinkedHashMap<>();
    void check(String id,String expectation,Scenario scenario) {
        if(cases.stream().anyMatch(c->id.equals(c.get("id"))))throw new IllegalArgumentException("Duplicate scenario: "+id);
        var row=new LinkedHashMap<String,Object>();row.put("id",id);row.put("expectation",expectation);
        try{row.put("measurements",scenario.run());row.put("status","pass");}
        catch(Exception e){row.put("status","fail");row.put("error",e.getMessage()==null?e.toString():e.getMessage());}
        cases.add(row);
    }
    static void require(boolean condition,String detail){if(!condition)throw new IllegalStateException(detail);}
    boolean passed(){return !cases.isEmpty()&&cases.stream().allMatch(c->"pass".equals(c.get("status")));}
    Map<String,Object> data(){return Map.of("schema_version",1,"suite_version","simulator-validation-v1","created_utc",Instant.now().toString(),"passed",passed(),"metadata",metadata,"cases",cases,"scope","Synthetic software/native physics checks; no measured hardware accuracy and no model review approval.");}
    void write(Path folder)throws Exception {
        ProfileIO.save(folder.resolve("report.json"),data());
        var md=new StringBuilder("# Simulator validation\n\n").append(passed()?"PASS":"FAIL").append(" — ").append(cases.stream().filter(c->"pass".equals(c.get("status"))).count()).append(" / ").append(cases.size()).append(" scenarios.\n\nSynthetic software/native physics checks; no measured hardware accuracy or model approval.\n\n## Configuration\n\n```json\n").append(ProfileIO.json(metadata)).append("\n```\n\n");
        for(var row:cases)md.append("## ").append(row.get("id")).append(" — ").append(row.get("status")).append("\n\nExpected: ").append(row.get("expectation")).append("\n\n```json\n").append(ProfileIO.json(row.getOrDefault("measurements",Map.of("error",row.getOrDefault("error",""))))).append("\n```\n\n");
        Files.writeString(folder.resolve("report.md"),md);
    }
}
