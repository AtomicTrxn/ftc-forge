package simrunner;

import com.jme3.bullet.*;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;
import java.nio.file.*;
import java.util.*;

/** Configurable practice objectives on native body bounds. Never claims referee equivalence. */
final class FieldBehavior implements PhysicsTickListener,AutoCloseable {
    record Rule(String id,String alliance,String mode,String containment,Set<String> types,Vector3f min,Vector3f max,
                int points,double from,double until) { }
    record Event(double seconds,String rule,String object,int points) { }
    private final PhysicsWorld world;
    private final List<Rule> rules;
    private final double auto,transition,teleop,duration;
    private final boolean piecesEnabled;
    private final Map<String,Set<String>> credited=new LinkedHashMap<>(),occupied=new LinkedHashMap<>();
    private final List<Event> events=new ArrayList<>();
    private double seconds;
    private boolean running,closed,eventsTruncated;
    FieldBehavior(PhysicsWorld world,Map<String,Object> config,boolean piecesEnabled) {
        this.world=world;this.piecesEnabled=piecesEnabled;validate(config);
        var clock=FieldPackage.map(config.getOrDefault("clock",Map.of()));
        auto=SceneSensorConfig.n(clock,"auto_s",30,0,600);transition=SceneSensorConfig.n(clock,"transition_s",8,0,600);teleop=SceneSensorConfig.n(clock,"teleop_s",120,0,600);duration=auto+transition+teleop;
        rules=parseRules(config,duration);reset();world.space().addTickListener(this);
    }
    static void validate(Map<String,Object> config) {
        if(config.isEmpty())return;
        if(FieldPackage.num(config,"schema_version")!=1)throw new IllegalArgumentException("Field behavior requires schema_version 1");
        if(!Set.of("schema_version","clock","rules","tags","colors","layout_reference").containsAll(config.keySet()))throw new IllegalArgumentException("Unknown field behavior setting");
        var clock=FieldPackage.map(config.getOrDefault("clock",Map.of()));if(!Set.of("auto_s","transition_s","teleop_s").containsAll(clock.keySet()))throw new IllegalArgumentException("Unknown match clock setting");
        double duration=0;for(String key:List.of("auto_s","transition_s","teleop_s"))duration+=SceneSensorConfig.n(clock,key,key.equals("auto_s")?30:key.equals("transition_s")?8:120,0,600);
        if(duration<=0)throw new IllegalArgumentException("Match duration must be positive");parseRules(config,duration);
        if(config.containsKey("layout_reference"))FieldLayoutAudit.validate(FieldPackage.map(config.get("layout_reference")));
    }
    private static List<Rule> parseRules(Map<String,Object> config,double duration) {
        var raw=FieldPackage.maps(config.getOrDefault("rules",List.of()));if(raw.size()>128)throw new IllegalArgumentException("At most 128 scoring rules");
        var rules=new ArrayList<Rule>();var ids=new HashSet<String>();
        for(var r:raw){if(!Set.of("id","alliance","mode","containment","types","min_xyz_m","max_xyz_m","points","from_s","until_s").containsAll(r.keySet()))throw new IllegalArgumentException("Unknown scoring rule setting");
            String id=FieldPackage.str(r,"id"),alliance=FieldPackage.str(r,"alliance"),mode=FieldPackage.str(r,"mode");String containment=(String)r.getOrDefault("containment","center");
            if(!ids.add(id)||!Set.of("red","blue","neutral").contains(alliance)||!Set.of("occupancy","entry_once","snapshot").contains(mode)||!Set.of("center","partial","full").contains(containment))throw new IllegalArgumentException("Invalid/duplicate scoring rule identity, alliance, mode or containment");
            if(!(r.get("types") instanceof List<?> t)||t.isEmpty()||t.stream().anyMatch(x->!(x instanceof String s)||s.isBlank())||t.size()>64)throw new IllegalArgumentException("Scoring types requires 1..64 names; robot denotes chassis");
            var lo=FieldPackage.pos(r.get("min_xyz_m"));var hi=FieldPackage.pos(r.get("max_xyz_m"));
            // The URDF Y coordinate maps to negative world Z: normalize each axis after conversion.
            var min=new Vector3f(Math.min(lo.x,hi.x),Math.min(lo.y,hi.y),Math.min(lo.z,hi.z));var max=new Vector3f(Math.max(lo.x,hi.x),Math.max(lo.y,hi.y),Math.max(lo.z,hi.z));
            if(min.x==max.x||min.y==max.y||min.z==max.z)throw new IllegalArgumentException("Scoring regions must have positive volume");
            double points=SceneSensorConfig.n(r,"points",1,0,1000),from=SceneSensorConfig.n(r,"from_s",0,0,duration),until=SceneSensorConfig.n(r,"until_s",duration,0,duration);if(points!=(int)points||until<=from)throw new IllegalArgumentException("Scoring points must be integer and window must increase");
            rules.add(new Rule(id,alliance,mode,containment,Set.copyOf((List<String>)t),min,max,(int)points,from,until));
        }return List.copyOf(rules);
    }
    synchronized void toggle(){if(seconds<duration)running=!running;}
    synchronized void reset(){running=false;seconds=0;events.clear();eventsTruncated=false;credited.clear();occupied.clear();for(var rule:rules){credited.put(rule.id,new HashSet<>());occupied.put(rule.id,new HashSet<>());}}
    synchronized String status(){return String.format(Locale.ROOT,"Practice scoring | %s %.1f s%s | red %d blue %d neutral %d | M: start/pause | K: export",phase(),seconds,running?"":" (paused)",score("red"),score("blue"),score("neutral"));}
    synchronized String phase(){return seconds>=duration?"FINISHED":seconds<auto?"AUTO":seconds<auto+transition?"TRANSITION":"TELEOP";}
    synchronized int score(String alliance){return rules.stream().filter(r->r.alliance.equals(alliance)).mapToInt(r->r.points*(r.mode.equals("occupancy")?occupied.get(r.id).size():credited.get(r.id).size())).sum();}
    private Set<String> inside(Rule rule) {
        var ids=new HashSet<String>();if(rule.types.contains("robot")&&contains(rule,world.chassisBody()))ids.add("robot");
        if(piecesEnabled)for(var piece:world.gamePieces())if(rule.types.contains(piece.type())&&contains(rule,piece.body()))ids.add(piece.id());return ids;
    }
    private boolean contains(Rule r,PhysicsRigidBody body) {
        var bounds=body.boundingBox(null);var lo=bounds.getMin(null);var hi=bounds.getMax(null);var center=body.getPhysicsLocation();
        return switch(r.containment){case "partial"->hi.x>=r.min.x&&lo.x<=r.max.x&&hi.y>=r.min.y&&lo.y<=r.max.y&&hi.z>=r.min.z&&lo.z<=r.max.z;case "full"->lo.x>=r.min.x&&hi.x<=r.max.x&&lo.y>=r.min.y&&hi.y<=r.max.y&&lo.z>=r.min.z&&hi.z<=r.max.z;default->center.x>=r.min.x&&center.x<=r.max.x&&center.y>=r.min.y&&center.y<=r.max.y&&center.z>=r.min.z&&center.z<=r.max.z;};
    }
    private void event(Event e){if(events.size()<10000)events.add(e);else eventsTruncated=true;}
    public void prePhysicsTick(PhysicsSpace space,float dt) { }
    public synchronized void physicsTick(PhysicsSpace space,float dt) {
        if(!running||closed)return;double before=seconds;seconds=Math.min(duration,seconds+dt);
        for(var rule:rules){boolean active=seconds>=rule.from&&seconds<rule.until;boolean ending=before<rule.until&&seconds>=rule.until;
            if(rule.mode.equals("snapshot")){if(ending){var set=inside(rule);credited.get(rule.id).addAll(set);for(String id:set)event(new Event(rule.until,rule.id,id,rule.points));}continue;}
            if(active||ending){var set=inside(rule);if(rule.mode.equals("occupancy")){var previous=occupied.get(rule.id);for(String id:set)if(!previous.contains(id))event(new Event(seconds,rule.id,id,rule.points));for(String id:previous)if(!set.contains(id))event(new Event(seconds,rule.id,id,-rule.points));occupied.put(rule.id,set);}else for(String id:set)if(credited.get(rule.id).add(id))event(new Event(seconds,rule.id,id,rule.points));}
        }if(seconds>=duration)running=false;
    }
    private static List<Float> xyz(Vector3f p){return List.of(p.x,-p.z,p.y);}
    synchronized Map<String,Object> report(){
        var rows=new ArrayList<Map<String,Object>>();for(var rule:rules){var row=new LinkedHashMap<String,Object>();row.put("rule",rule.id);row.put("alliance",rule.alliance);row.put("mode",rule.mode);row.put("containment",rule.containment);row.put("types",rule.types.stream().sorted().toList());row.put("min_xyz_m",List.of(rule.min.x,-rule.max.z,rule.min.y));row.put("max_xyz_m",List.of(rule.max.x,-rule.min.z,rule.max.y));row.put("from_s",rule.from);row.put("until_s",rule.until);row.put("objects",(rule.mode.equals("occupancy")?occupied.get(rule.id):credited.get(rule.id)).stream().sorted().toList());row.put("points_per_object",rule.points);rows.add(row);}
        var report=new LinkedHashMap<String,Object>();report.put("schema_version",1);report.put("scope","Configurable practice scoring; robot regions use chassis bounds; no penalties or referee decisions");report.put("seconds",seconds);report.put("phase",phase());report.put("pieces_enabled",piecesEnabled);report.put("clock",Map.of("auto_s",auto,"transition_s",transition,"teleop_s",teleop));report.put("scores",Map.of("red",score("red"),"blue",score("blue"),"neutral",score("neutral")));report.put("rules",rows);report.put("events_truncated",eventsTruncated);report.put("events",events.stream().map(e->Map.of("seconds",e.seconds,"rule",e.rule,"object",e.object,"points",e.points)).toList());report.put("robot_xyz_m",xyz(world.chassisBody().getPhysicsLocation()));report.put("pieces",world.gamePieces().stream().map(p->Map.of("id",p.id(),"type",p.type(),"xyz_m",xyz(p.body().getPhysicsLocation()))).toList());return report;
    }
    synchronized Path export(Path folder)throws Exception{Files.createDirectories(folder);Path file=folder.resolve("field-score-"+UUID.randomUUID()+".json");Files.writeString(file,ProfileIO.json(report()));return file;}
    public synchronized void close(){if(!closed){closed=true;world.space().removeTickListener(this);}}
}
