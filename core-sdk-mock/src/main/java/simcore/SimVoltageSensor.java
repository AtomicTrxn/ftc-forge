package simcore;

import com.qualcomm.hardware.lynx.LynxModule;

/**
 * A simulated hub: a LynxModule (so team code can call getAll(LynxModule.class) for bulk
 * caching, as the Road Runner quickstart does) whose VoltageSensor reading reflects the
 * real, shared battery-sag voltage HardwareMapBuilder computes each tick (R4), not a constant.
 */
public class SimVoltageSensor extends LynxModule {
    public SimVoltageSensor(String name) { super(name); }

    @Override public String getConnectionInfo() { return "Simulated hub \"" + getDeviceName() + "\""; }
}
