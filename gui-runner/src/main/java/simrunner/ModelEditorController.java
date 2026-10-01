package simrunner;
import java.nio.file.*;
import java.util.*;
import simcore.MiniJson;

/** Testable editor operations; runs preparation outside Swing's event thread. */
final class ModelEditorController {
    final Path library;
    ModelEditorController(Path library){this.library=library.toAbsolutePath();}
    Object run(String... arguments)throws Exception {
        Path tool=Path.of("tools/model_preparation.py");if(!Files.exists(tool))tool=Path.of("../tools/model_preparation.py");
        var command=new ArrayList<String>(List.of("python3",tool.toAbsolutePath().toString()));command.addAll(List.of(arguments));
        var p=new ProcessBuilder(command).redirectErrorStream(true).start();String log=new String(p.getInputStream().readAllBytes(),java.nio.charset.StandardCharsets.UTF_8);
        if(p.waitFor()!=0)throw new IllegalArgumentException(log.strip());
        int line=log.strip().lastIndexOf('\n');return MiniJson.parseObject(log.strip().substring(line+1)).get("result");
    }
    List<Path> revisions()throws Exception {
        if(!Files.exists(library.resolve("models")))return List.of();
        try(var walk=Files.walk(library.resolve("models"))){return walk.filter(p->p.getFileName().toString().equals("profile.json")).sorted(Comparator.comparingLong((Path p)->{try{return Files.getLastModifiedTime(p).toMillis();}catch(Exception e){return 0;}}).reversed()).toList();}
    }
    static Process javaProcess(Class<?> cls,List<String> args,boolean graphics)throws Exception {return new ProcessBuilder(javaCommand(cls,args,graphics)).inheritIO().start();}
    static Process loggedProcess(Class<?> cls,List<String> args,Path log)throws Exception {Files.createDirectories(log.toAbsolutePath().getParent());return new ProcessBuilder(javaCommand(cls,args,false)).redirectErrorStream(true).redirectOutput(log.toFile()).start();}
    private static List<String> javaCommand(Class<?> cls,List<String> args,boolean graphics) {
        var command=new ArrayList<String>();command.add(Path.of(System.getProperty("java.home"),"bin/java").toString());
        if(graphics&&System.getProperty("os.name").contains("Mac"))command.add("-XstartOnFirstThread");
        command.addAll(List.of("-Djava.awt.headless=true","-Xmx2g","-cp",System.getProperty("java.class.path"),cls.getName()));command.addAll(args);return command;
    }
}
