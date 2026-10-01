package simrunner;

import com.jme3.math.*;
import simcore.MiniJson;
import java.nio.file.*;
import java.util.*;

/** Bounded prepared-field manifest. Meter values are never fitted to robot bounds. */
final class FieldPackage {
    final Path directory;
    final Path sourceUrdf;
    final String name;
    final Vector3f halfExtents;
    final Map<String,Map<String,Object>> meshes;
    final List<Map<String,Object>> instances, hives;
    final List<String> approximations;
    @SuppressWarnings("unchecked")
    FieldPackage(Path path) throws Exception {
        path=path.toAbsolutePath().normalize();
        if(Files.isDirectory(path))path=path.resolve("field.json");
        if(!Files.isRegularFile(path)||Files.size(path)>10_000_000)throw new IllegalArgumentException("Missing or oversized prepared field: "+path+". Run tools/prepare_biobuzz.py first.");
        directory=path.getParent().toRealPath();
        var data=MiniJson.parseObject(Files.readString(path));
        if(num(data,"version")!=1 || !"m".equals(data.get("units")))throw new IllegalArgumentException("Field package must be version 1 in meters");
        name=str(data,"name");
        sourceUrdf=resolve(str(data,"urdf"));
        var half=vector(data.get("field_half_extents_m"),2);
        if(half[0]<1.7||half[0]>1.9||half[1]<1.7||half[1]>1.9||Math.abs(num(data,"floor_top_m"))>1e-6)
            throw new IllegalArgumentException("Field origin/dimensions require preparation in meters");
        halfExtents=new Vector3f((float)half[0],0,(float)half[1]);
        meshes=(Map<String,Map<String,Object>>)data.get("meshes");
        instances=maps(data.get("instances"));hives=maps(data.get("hives"));
        approximations=((List<?>)data.get("approximations")).stream().map(Object::toString).toList();
        if(meshes==null||meshes.size()>200||instances.size()>2000||hives.size()!=2)throw new IllegalArgumentException("Field manifest exceeds supported profile");
        Set<String> ids=new HashSet<>();Set<String> owners=new HashSet<>();
        owners.add("fixed");owners.add("piece");
        for(var h:hives){if(!owners.add(str(h,"id")))throw new IllegalArgumentException("Duplicate HIVE");transform(h);var axis=pos(h.get("axis"));if(Math.abs(axis.length()-1)>.001||num(h,"mass_kg")<=0||num(h,"lower_rad")>=num(h,"upper_rad")||num(h,"damping")<0||num(h,"damping")>1)throw new IllegalArgumentException("Invalid HIVE axis/mass/limits/damping");}
        for(var m:meshes.values()) {
            resolve(str(m,"file"));
            if(num(m,"triangles")<=0||num(m,"triangles")>100_000)throw new IllegalArgumentException("Prepared visual exceeds triangle budget");
        }
        for(String key:meshes.keySet())if(!Path.of(key).getFileName().toString().equals(key))throw new IllegalArgumentException("Mesh identity must be a filename");
        for(var i:instances) {
            if(!ids.add(str(i,"id"))||!meshes.containsKey(str(i,"mesh"))||!owners.contains(str(i,"owner")))throw new IllegalArgumentException("Invalid field instance identity/owner");
            transform(i);for(double color:vector(i.get("rgba"),4))if(color<0||color>1)throw new IllegalArgumentException("Invalid field color");
            if(str(i,"owner").equals("piece")) {
                if(!Set.of("pollen","red_nectar","blue_nectar").contains(str(i,"category")))throw new IllegalArgumentException("Unknown field piece type");
                if(num(i,"radius_m")<.03||num(i,"radius_m")>.06||num(i,"mass_kg")<=0)throw new IllegalArgumentException("Invalid piece size/mass");
                var bounds=(List<?>)meshes.get(str(i,"mesh")).get("bounds_m");var lo=vector(bounds.get(0),3);var hi=vector(bounds.get(1),3);var center=vector(i.get("mesh_center_m"),3);double diameter=0;
                for(int axis=0;axis<3;axis++){diameter=Math.max(diameter,hi[axis]-lo[axis]);if(Math.abs(center[axis]-(lo[axis]+hi[axis])/2)>.00001)throw new IllegalArgumentException("Piece collision center disagrees with mesh bounds");}
                if(Math.abs(diameter/2-num(i,"radius_m"))>.00001)throw new IllegalArgumentException("Piece collision radius disagrees with mesh dimensions");
                var p=transform(i).getTranslation();float radius=(float)num(i,"radius_m");
                if(p.y<radius||Math.abs(p.x)+radius>halfExtents.x||Math.abs(p.z)+radius>halfExtents.z)throw new IllegalArgumentException("Practice piece starts below floor/outside perimeter: "+i.get("id"));
            }
        }
        var pieces=instances.stream().filter(i->str(i,"owner").equals("piece")).toList();
        for(int a=0;a<pieces.size();a++)for(int b=a+1;b<pieces.size();b++)
            if(transform(pieces.get(a)).getTranslation().distance(transform(pieces.get(b)).getTranslation()) < num(pieces.get(a),"radius_m")+num(pieces.get(b),"radius_m"))throw new IllegalArgumentException("Overlapping practice pieces");
    }
    Path resolve(String file) throws Exception {
        Path result=directory.resolve(file).normalize();
        if(!result.startsWith(directory)||!Files.isRegularFile(result)||!result.toRealPath().startsWith(directory))throw new IllegalArgumentException("Field mesh missing or escapes package: "+file);
        return result;
    }
    static String str(Map<String,Object> m,String key) {if(!(m.get(key) instanceof String s)||s.isBlank())throw new IllegalArgumentException("Field requires text "+key);return s;}
    static double num(Map<String,Object> m,String key) {if(!(m.get(key) instanceof Number n)||!Double.isFinite(n.doubleValue()))throw new IllegalArgumentException("Field requires finite "+key);return n.doubleValue();}
    static double[] vector(Object value,int length) {if(!(value instanceof List<?> v)||v.size()!=length)throw new IllegalArgumentException("Field requires vector length "+length);double[] out=new double[length];for(int i=0;i<length;i++){if(!(v.get(i) instanceof Number n)||!Double.isFinite(n.doubleValue()))throw new IllegalArgumentException("Nonfinite field vector");out[i]=n.doubleValue();}return out;}
    @SuppressWarnings("unchecked") static List<Map<String,Object>> maps(Object value) {if(!(value instanceof List<?> list)||list.stream().anyMatch(x->!(x instanceof Map)))throw new IllegalArgumentException("Field requires object array");return (List<Map<String,Object>>)value;}
    @SuppressWarnings("unchecked") static Map<String,Object> map(Object value) {if(!(value instanceof Map))throw new IllegalArgumentException("Field requires object");return (Map<String,Object>)value;}
    static Vector3f pos(Object value) {var p=ImportedRobotScene.position(vector(value,3));if(!Float.isFinite(p.x)||!Float.isFinite(p.y)||!Float.isFinite(p.z)||p.length()>1000)throw new IllegalArgumentException("Field position exceeds metric budget");return p;}
    static Transform transform(Map<String,Object> m) {return new Transform(pos(m.get("xyz_m")),ImportedRobotScene.rotation(vector(m.get("rpy_rad"),3)));}
}
