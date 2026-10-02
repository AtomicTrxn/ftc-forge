package simrunner;

import simcore.RobotUrdf;
import java.nio.file.*;
import java.util.*;

/** Collision declaration coverage at the same rigid-body boundaries used by physics.
 * Presence of a declaration is not a geometric or material accuracy certificate.
 */
final class CollisionAudit {
    record Link(String name, String body, String treatment, int declarations) { }
    record Body(String name, int declarations, List<String> members, String status, String reason) { }
    final List<Link> links;
    final List<Body> bodies;
    final List<String> errors;
    private CollisionAudit(List<Link> links,List<Body> bodies,List<String> errors) {
        this.links=List.copyOf(links);this.bodies=List.copyOf(bodies);this.errors=List.copyOf(errors);
    }
    static CollisionAudit inspect(ImportedRobotScene scene) {
        scene.groupParts();
        List<Link> links=new ArrayList<>();List<Body> bodies=new ArrayList<>();List<String> errors=new ArrayList<>();
        Set<String> owners=new TreeSet<>(scene.owners.values());
        if(scene.driveContacts!=null)for(String joint:scene.driveContacts.wheelFriction().keySet())
            if(!scene.wheelJoints.containsValue(joint))errors.add("Wheel friction target is not a configured drive wheel: "+joint+". Choose a current wheel or remove its saved override.");
        if(scene.wheelContactsEnabled())for(String joint:new TreeSet<>(scene.wheelJoints.values())) {
            var members=scene.wheelJoints.entrySet().stream().filter(e->e.getValue().equals(joint)).map(Map.Entry::getKey).toList();
            if(members.stream().anyMatch(n->!scene.owners.get(n).equals(scene.urdf.rootLink)))errors.add("Drive wheel '"+joint+"' must belong to the welded chassis for native support. Review its parent joints.");
            if(members.stream().allMatch(n->scene.urdf.links.get(n).collisions().isEmpty()))errors.add("Drive wheel '"+joint+"' needs reviewed collision geometry for native support.");
        }
        for(var omission:scene.collisionOmissions.entrySet()) {
            if(!owners.contains(omission.getKey()))errors.add("Unknown rigid body in collision_omissions: "+omission.getKey());
            if(omission.getValue()==null || omission.getValue().isBlank())errors.add("Collision omission requires a reason: "+omission.getKey());
            if(omission.getKey().equals(scene.urdf.rootLink))errors.add("The chassis cannot be omitted from collision: "+omission.getKey());
        }
        for(String owner:owners) {
            var members=scene.urdf.links.keySet().stream().filter(n->scene.owners.get(n).equals(owner)).sorted().toList();
            int count=members.stream().filter(n->scene.wheelContactsEnabled()||!scene.wheelLinks.contains(n)).mapToInt(n->scene.urdf.links.get(n).collisions().size()).sum();
            String reason=scene.collisionOmissions.get(owner);
            String status=count>0?"declared":reason==null?"missing":"intentional_noncontact";
            if(count==0 && reason==null)errors.add("Rigid body '"+owner+"' has no effective collision geometry. Add shapes to it or its fixed children; use previewRobot and auditCollisions to review the import.");
            if(count>0 && reason!=null)errors.add("Redundant collision omission on collidable body: "+owner);
            bodies.add(new Body(owner,count,members,status,reason));
            for(String name:members) {
                RobotUrdf.Link link=scene.urdf.links.get(name);String treatment;
                if(scene.wheelLinks.contains(name)) {
                    treatment=scene.wheelContactsEnabled()?"native_drive_wheel_contacts":scene.tireContacts?"tire_contact_model":"drive_wheel_ballast";
                }
                else if(scene.flexibleIntake!=null && scene.flexibleIntake.links().contains(name))treatment="flexible_contacts_plus_rigid_proxy";
                else if(!link.collisions().isEmpty())treatment="declared_on_link";
                else if(link.visuals().isEmpty())treatment="assembly_or_inertial_frame";
                else treatment=count>0?"aggregate_proxy_review_required":status;
                links.add(new Link(name,owner,treatment,link.collisions().size()));
            }
        }
        return new CollisionAudit(links,bodies,errors);
    }
    boolean usable(){return errors.isEmpty();}
    void requireUsable(){if(!usable())throw new IllegalArgumentException(String.join("\n",errors));}
    String summary(){return bodies.size()+" rigid bodies | "+bodies.stream().filter(b->b.status().equals("missing")).count()+" missing | "+bodies.stream().filter(b->b.status().equals("intentional_noncontact")).count()+" intentional noncontact | "+links.stream().filter(l->l.treatment().equals("aggregate_proxy_review_required")).count()+" aggregate-only visual links";}
    void write(Path file,Path source) throws Exception {
        file=file.toAbsolutePath().normalize();
        if(!file.getFileName().toString().endsWith(".json") || Files.isSymbolicLink(file)
            || file.equals(source.toAbsolutePath().normalize()) || Files.exists(file)&&Files.isSameFile(file,source))
            throw new IllegalArgumentException("Collision report must be a separate .json file, not source CAD or a symlink");
        List<Object> linkRows=new ArrayList<>(),bodyRows=new ArrayList<>();
        for(var l:links)linkRows.add(Map.of("link",l.name(),"body",l.body(),"treatment",l.treatment(),"collision_declarations",l.declarations()));
        for(var b:bodies){var row=new LinkedHashMap<String,Object>();row.put("body",b.name());row.put("status",b.status());row.put("collision_declarations",b.declarations());row.put("members",b.members());row.put("reason",b.reason());bodyRows.add(row);}
        var report=new LinkedHashMap<String,Object>();report.put("version",1);report.put("source_urdf",source.toAbsolutePath().toString());report.put("coverage_usable",usable());report.put("scope","Declaration coverage only. Geometry, clearances, mesh decomposition, mass/inertia and materials require separate validation.");report.put("summary",summary());report.put("errors",errors);report.put("bodies",bodyRows);report.put("links",linkRows);
        Files.createDirectories(file.getParent());Files.writeString(file,json(report)+"\n");
    }
    static String json(Object value) {
        if(value==null)return "null";
        if(value instanceof String s){var out=new StringBuilder("\"");for(char c:s.toCharArray())switch(c){case '"'->out.append("\\\"");case '\\'->out.append("\\\\");case '\n'->out.append("\\n");case '\r'->out.append("\\r");case '\t'->out.append("\\t");default->{if(c<32)out.append(String.format("\\u%04x",(int)c));else out.append(c);}}return out.append('"').toString();}
        if(value instanceof Number || value instanceof Boolean)return value.toString();
        if(value instanceof Map<?,?> map)return "{"+String.join(",",map.entrySet().stream().map(e->json(e.getKey().toString())+":"+json(e.getValue())).toList())+"}";
        if(value instanceof List<?> list)return "["+String.join(",",list.stream().map(CollisionAudit::json).toList())+"]";
        throw new IllegalArgumentException("Unsupported report value");
    }
}
