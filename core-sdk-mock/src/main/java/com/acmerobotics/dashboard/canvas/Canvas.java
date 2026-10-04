package com.acmerobotics.dashboard.canvas;

/**
 * Drawing-surface shim for TelemetryPacket.fieldOverlay(). Calls are accepted (fluently, as the
 * real Canvas does) and counted, but nothing is rendered -- there is no dashboard web UI here.
 */
public class Canvas {
    private int operations;

    private Canvas op() { operations++; return this; }

    /** Number of drawing calls recorded since creation; lets tests prove an overlay was drawn. */
    public int getOperationCount() { return operations; }

    public Canvas setAlpha(double alpha) { return op(); }
    public Canvas setFill(String color) { return op(); }
    public Canvas setStroke(String color) { return op(); }
    public Canvas setStrokeWidth(int width) { return op(); }
    public Canvas setRotation(double radians) { return op(); }
    public Canvas setTranslation(double x, double y) { return op(); }
    public Canvas setScale(double scaleX, double scaleY) { return op(); }
    public Canvas fillCircle(double x, double y, double radius) { return op(); }
    public Canvas strokeCircle(double x, double y, double radius) { return op(); }
    public Canvas fillRect(double x, double y, double width, double height) { return op(); }
    public Canvas strokeRect(double x, double y, double width, double height) { return op(); }
    public Canvas strokeLine(double x1, double y1, double x2, double y2) { return op(); }
    public Canvas fillPolygon(double[] xPoints, double[] yPoints) { return op(); }
    public Canvas strokePolygon(double[] xPoints, double[] yPoints) { return op(); }
    public Canvas strokePolyline(double[] xPoints, double[] yPoints) { return op(); }
    public Canvas fillText(String text, double x, double y, String font, double theta) { return op(); }
    public Canvas drawGrid(double x, double y, double width, double height, int numTicksX, int numTicksY) { return op(); }
}
