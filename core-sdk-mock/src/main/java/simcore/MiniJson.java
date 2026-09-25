package simcore;

import java.util.*;

/**
 * Minimal recursive-descent JSON reader -- objects, arrays, strings, numbers, booleans, null.
 * DEVIATION (documented): no Gradle/dependency resolution is wired up in this repo yet (R3's
 * scope), so this avoids requiring an external JSON library just to parse sim.config and the
 * preset robot configs. Replace with a real library (e.g. org.json) once a real build exists.
 */
public final class MiniJson {
    private final String s;
    private int pos;

    private MiniJson(String s) { this.s = s; }

    @SuppressWarnings("unchecked")
    public static Map<String, Object> parseObject(String json) {
        MiniJson p = new MiniJson(json);
        p.skipWs();
        Object result = p.parseValue();
        if (!(result instanceof Map)) throw new IllegalArgumentException("Expected a JSON object at top level");
        return (Map<String, Object>) result;
    }

    private Object parseValue() {
        skipWs();
        char c = s.charAt(pos);
        if (c == '{') return parseMap();
        if (c == '[') return parseArray();
        if (c == '"') return parseString();
        if (c == 't') { expect("true"); return Boolean.TRUE; }
        if (c == 'f') { expect("false"); return Boolean.FALSE; }
        if (c == 'n') { expect("null"); return null; }
        return parseNumber();
    }

    private Map<String, Object> parseMap() {
        Map<String, Object> map = new LinkedHashMap<>();
        pos++; // {
        skipWs();
        if (s.charAt(pos) == '}') { pos++; return map; }
        while (true) {
            skipWs();
            String key = parseString();
            skipWs();
            if (s.charAt(pos) != ':') throw new IllegalArgumentException("Expected ':' at " + pos);
            pos++;
            Object value = parseValue();
            map.put(key, value);
            skipWs();
            char c = s.charAt(pos);
            if (c == ',') { pos++; continue; }
            if (c == '}') { pos++; break; }
            throw new IllegalArgumentException("Expected ',' or '}' at " + pos);
        }
        return map;
    }

    private List<Object> parseArray() {
        List<Object> list = new ArrayList<>();
        pos++; // [
        skipWs();
        if (s.charAt(pos) == ']') { pos++; return list; }
        while (true) {
            list.add(parseValue());
            skipWs();
            char c = s.charAt(pos);
            if (c == ',') { pos++; continue; }
            if (c == ']') { pos++; break; }
            throw new IllegalArgumentException("Expected ',' or ']' at " + pos);
        }
        return list;
    }

    private String parseString() {
        if (s.charAt(pos) != '"') throw new IllegalArgumentException("Expected string at " + pos);
        pos++;
        StringBuilder sb = new StringBuilder();
        while (s.charAt(pos) != '"') {
            char c = s.charAt(pos);
            if (c == '\\') {
                pos++;
                char esc = s.charAt(pos);
                switch (esc) {
                    case 'n': sb.append('\n'); break;
                    case 't': sb.append('\t'); break;
                    case '"': sb.append('"'); break;
                    case '\\': sb.append('\\'); break;
                    case '/': sb.append('/'); break;
                    default: sb.append(esc);
                }
            } else {
                sb.append(c);
            }
            pos++;
        }
        pos++; // closing quote
        return sb.toString();
    }

    private Object parseNumber() {
        int start = pos;
        while (pos < s.length() && "-+.eE0123456789".indexOf(s.charAt(pos)) >= 0) pos++;
        String numStr = s.substring(start, pos);
        if (numStr.contains(".") || numStr.toLowerCase().contains("e")) return Double.parseDouble(numStr);
        return Long.parseLong(numStr);
    }

    private void expect(String literal) {
        if (!s.regionMatches(pos, literal, 0, literal.length())) {
            throw new IllegalArgumentException("Expected '" + literal + "' at " + pos);
        }
        pos += literal.length();
    }

    private void skipWs() {
        while (pos < s.length() && Character.isWhitespace(s.charAt(pos))) pos++;
    }
}
