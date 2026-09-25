package simcore;

import org.w3c.dom.*;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Parser for the real FTC robot-configuration XML format (per R3's research and worked
 * example): root element <Robot type="FirstInspires-FTC">, hub modules containing named,
 * ported device tags. This establishes *what hardware exists and what it's called* --
 * gear ratio/SKU is carried separately by the preset config (R3/R4 found the XML can't
 * express it).
 *
 * DEVIATION (documented): R1/R3 both flagged that this format's full tag vocabulary is not
 * officially documented. This parser handles the tags in this project's own worked example
 * (LynxUsbDevice/LynxModule + goBILDA5202SeriesMotor/Servo/RevIMU) via a name-convention
 * device-type resolver, not an exhaustive real-world tag list -- expand as real exported
 * configs are tested against it (flagged as an open risk in R1/R3, not resolved here).
 */
public class RobotConfigXml {

    public static class DeviceEntry {
        public final String tag;
        public final String name;
        public final String port;
        public DeviceEntry(String tag, String name, String port) {
            this.tag = tag;
            this.name = name;
            this.port = port;
        }
    }

    public final List<DeviceEntry> devices = new ArrayList<>();
    public final List<String> hubNames = new ArrayList<>();

    public static RobotConfigXml parse(File file) throws Exception {
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        // Harden against XXE per standard practice, even though these are trusted local files.
        factory.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
        DocumentBuilder builder = factory.newDocumentBuilder();
        Document doc = builder.parse(file);

        RobotConfigXml config = new RobotConfigXml();
        NodeList hubs = doc.getElementsByTagName("LynxModule");
        for (int i = 0; i < hubs.getLength(); i++) {
            Element hub = (Element) hubs.item(i);
            config.hubNames.add(hub.getAttribute("name"));
            NodeList children = hub.getChildNodes();
            for (int j = 0; j < children.getLength(); j++) {
                Node node = children.item(j);
                if (node.getNodeType() != Node.ELEMENT_NODE) continue;
                Element el = (Element) node;
                config.devices.add(new DeviceEntry(el.getTagName(), el.getAttribute("name"), el.getAttribute("port")));
            }
        }
        return config;
    }

    /** Resolves an XML tag name to a device-type category using the naming convention in R3's example. */
    public static DeviceType resolveType(String tag) {
        String t = tag.toLowerCase();
        if (t.contains("crservo")) return DeviceType.CR_SERVO;
        if (t.contains("servo")) return DeviceType.SERVO;
        if (t.contains("motor")) return DeviceType.MOTOR;
        if (t.contains("imu")) return DeviceType.IMU;
        if (t.contains("distance")) return DeviceType.DISTANCE_SENSOR;
        if (t.contains("color")) return DeviceType.COLOR_SENSOR;
        if (t.contains("touch")) return DeviceType.TOUCH_SENSOR;
        return DeviceType.UNKNOWN;
    }

    public enum DeviceType { MOTOR, SERVO, CR_SERVO, IMU, DISTANCE_SENSOR, COLOR_SENSOR, TOUCH_SENSOR, UNKNOWN }
}
