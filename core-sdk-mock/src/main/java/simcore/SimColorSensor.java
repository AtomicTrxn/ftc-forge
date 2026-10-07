package simcore;

import com.qualcomm.robotcore.hardware.ColorSensor;

/** Atomic RGBA scene sample; unconfigured devices retain black. */
public class SimColorSensor implements ColorSensor {
    private final String name;
    public SimColorSensor(String name) { this.name = name; }
    private volatile int[] rgba={0,0,0,0};
    private volatile double readCostMs;
    /** Time each channel read blocks the caller, modelling the I2C round trip. */
    public void setReadCostMs(double ms){if(!(ms>=0&&ms<=50))throw new IllegalArgumentException("Sensor read cost must be 0..50 ms");readCostMs=ms;}
    public void setColor(int red,int green,int blue,int alpha){for(int v:new int[]{red,green,blue,alpha})if(v<0||v>255)throw new IllegalArgumentException("Color channels must be 0..255");rgba=new int[]{red,green,blue,alpha};}

    @Override public int red() { BusCost.block(readCostMs); return rgba[0]; }
    @Override public int green() { BusCost.block(readCostMs); return rgba[1]; }
    @Override public int blue() { BusCost.block(readCostMs); return rgba[2]; }
    @Override public int alpha() { BusCost.block(readCostMs); return rgba[3]; }
    @Override public String getDeviceName() { return name; }
    @Override public String getConnectionInfo() { return "Simulated color sensor \"" + name + "\""; }
    @Override public void close() { }
}
