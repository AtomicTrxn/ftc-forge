package com.qualcomm.robotcore.util;

public class RobotLog {
    private RobotLog() {}

    public static void a(String format, Object... args) { System.out.println("[ASSERT] " + String.format(format, args)); }
    public static void e(String format, Object... args) { System.out.println("[ERROR] " + String.format(format, args)); }
    public static void w(String format, Object... args) { System.out.println("[WARN] " + String.format(format, args)); }
    public static void d(String format, Object... args) { System.out.println("[DEBUG] " + String.format(format, args)); }
    public static void ii(String tag, String format, Object... args) { System.out.println("[INFO:" + tag + "] " + String.format(format, args)); }
}
