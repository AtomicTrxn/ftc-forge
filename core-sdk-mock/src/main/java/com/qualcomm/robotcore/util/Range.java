package com.qualcomm.robotcore.util;

public class Range {
    private Range() {}

    public static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static int clip(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
}
