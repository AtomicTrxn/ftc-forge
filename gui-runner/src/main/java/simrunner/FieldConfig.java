package simrunner;

import java.util.Map;

/** Field source and piece mode are independent of the robot hardware/model. */
record FieldConfig(String source, String mode, String packagePath, String pieceSet, boolean fullDetail,
                   float friction, float restitution) {
    static FieldConfig defaults() { return new FieldConfig("generic", "field-only", null, "biobuzz", false, .6f, .15f); }
    static FieldConfig torusPractice() { return new FieldConfig("generic", "game-pieces", null, "torus", false, .6f, .15f); }
    static FieldConfig parse(Map<String,Object> values) {
        var d=defaults();
        return new FieldConfig(string(values,"source",d.source),string(values,"mode",d.mode),
            string(values,"package",null),string(values,"piece_set",d.pieceSet),
            bool(values,"full_detail",false),number(values,"friction",.6f),number(values,"restitution",.15f)).validated();
    }
    FieldConfig validated() {
        if (!source.equals("generic") && !source.equals("imported")) throw new IllegalArgumentException("field.source must be generic or imported");
        if (!mode.equals("field-only") && !mode.equals("game-pieces")) throw new IllegalArgumentException("field.mode must be field-only or game-pieces");
        if (!pieceSet.equals("biobuzz") && !pieceSet.equals("torus")) throw new IllegalArgumentException("field.piece_set must be biobuzz or torus");
        if(source.equals("imported") && (packagePath==null || packagePath.isBlank())) throw new IllegalArgumentException("Imported field requires field.package (prepared field.json)");
        if(source.equals("imported") && !pieceSet.equals("biobuzz")) throw new IllegalArgumentException("Imported BIOBUZZ field requires its biobuzz piece set");
        if(!Float.isFinite(friction)||friction<0||friction>2 || !Float.isFinite(restitution)||restitution<0||restitution>1)
            throw new IllegalArgumentException("Field material requires friction 0..2 and restitution 0..1");
        return this;
    }
    boolean hasPieces() { return mode.equals("game-pieces"); }
    boolean usesTorus() { return hasPieces() && pieceSet.equals("torus"); }
    FieldConfig withSource(String path) { return new FieldConfig(path.equals("generic")?"generic":"imported",mode,path.equals("generic")?null:path,path.equals("generic")?pieceSet:"biobuzz",fullDetail,friction,restitution).validated(); }
    FieldConfig withMode(String value) { return new FieldConfig(source,value,packagePath,pieceSet,fullDetail,friction,restitution).validated(); }
    private static String string(Map<String,Object> m,String k,String d) { if(!m.containsKey(k))return d;if(!(m.get(k) instanceof String s))throw new IllegalArgumentException("field."+k+" must be text");return s; }
    private static boolean bool(Map<String,Object> m,String k,boolean d) {if(!m.containsKey(k))return d;if(!(m.get(k) instanceof Boolean b))throw new IllegalArgumentException("field."+k+" must be boolean");return b;}
    private static float number(Map<String,Object> m,String k,float d) {if(!m.containsKey(k))return d;if(!(m.get(k) instanceof Number n))throw new IllegalArgumentException("field."+k+" must be numeric");return n.floatValue();}
}
