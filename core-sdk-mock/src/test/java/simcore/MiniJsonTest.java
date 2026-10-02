package simcore;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class MiniJsonTest {
    @Test void readsPythonPortableUnicodeAndControlEscapesWithoutChangingNotes() {
        String encoded="{\"note\":\"caf\\u00e9 \\u2014 \\ud83e\\udd16\\r\\n\\t\\b\\f\"}";
        assertEquals("café — 🤖\r\n\t\b\f",MiniJson.parseObject(encoded).get("note"));
    }
    @Test void rejectsMalformedEscapesInsteadOfSilentlyChangingSavedText() {
        assertThrows(IllegalArgumentException.class,()->MiniJson.parseObject("{\"note\":\"\\u00xz\"}"));
        assertThrows(IllegalArgumentException.class,()->MiniJson.parseObject("{\"note\":\"\\q\"}"));
    }
}
