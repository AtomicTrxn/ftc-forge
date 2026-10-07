package simcore;

/** A device that draws battery current, evaluated once per simulator tick. */
public interface ServoLoad {
    /**
     * Battery-side current at {@code timeMs}. {@code holdA} applies once the device has been commanded,
     * {@code activeA} while it is moving (or, for continuous-rotation servos, running), and movement
     * is considered to last {@code activeMs} after the command last changed.
     */
    double drawAmps(long timeMs, double holdA, double activeA, long activeMs);
}
