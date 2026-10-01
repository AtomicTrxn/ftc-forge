package simrunner;
import java.nio.file.*;
import java.util.*;

/** Small JSON writer and atomic settings save; parser remains the shared MiniJson. */
final class ProfileIO {
    static String json(Object o){
        if(o==null)return "null";
        if(o instanceof String s){var out=new StringBuilder("\"");for(char c:s.toCharArray())switch(c){case '"'->out.append("\\\"");case '\\'->out.append("\\\\");case '\n'->out.append("\\n");case '\r'->out.append("\\r");case '\t'->out.append("\\t");default->{if(c<32)out.append(String.format("\\u%04x",(int)c));else out.append(c);}}return out.append('"').toString();}
        if(o instanceof Boolean)return o.toString();
        if(o instanceof Number n){if(!Double.isFinite(n.doubleValue()))throw new IllegalArgumentException("Finite numbers required");return n.toString();}
        if(o instanceof Map<?,?> m){var a=new ArrayList<String>();m.forEach((k,v)->a.add(json(k.toString())+":"+json(v)));return "{"+String.join(",",a)+"}";}
        if(o instanceof List<?> l)return "["+String.join(",",l.stream().map(ProfileIO::json).toList())+"]";
        throw new IllegalArgumentException("Unsupported settings value");
    }
    static void save(Path path,Object o)throws Exception{Files.createDirectories(path.toAbsolutePath().getParent());Path temp=Files.createTempFile(path.toAbsolutePath().getParent(),".save-",".json");try{Files.writeString(temp,json(o)+"\n");Files.move(temp,path,StandardCopyOption.ATOMIC_MOVE,StandardCopyOption.REPLACE_EXISTING);}finally{Files.deleteIfExists(temp);}}
}
