package org.firstinspires.ftc.robotcore.external;

/** Mock of org.firstinspires.ftc.robotcore.external.Telemetry -- pure data-sink interface. */
public interface Telemetry {

    interface Item {
        Object setValue(Object value);
    }

    interface Line {
    }

    interface Log {
        void add(String entry);
    }

    Item addData(String caption, Object value);
    Item addData(String caption, String format, Object... args);
    Line addLine(String lineCaption);
    boolean update();
    void clear();
    void setAutoClear(boolean autoClear);
    void setMsTransmissionInterval(int ms);
    Log log();
}
