package com.team2813.lib.util;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightValues {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private final NetworkTableEntry ledMode = table.getEntry("ledMode");
    private final NetworkTableEntry stream = table.getEntry("stream");

    private final DoubleArrayValue fieldLocation = new DoubleArrayValue(table, "botpose");
    private final DoubleValue horizonalOffset = new DoubleValue(table, "tx");
    private final DoubleValue verticalOffset = new DoubleValue(table, "ty");
    private final DoubleValue hasTargets = new DoubleValue(table, "tv");

    public static enum LedState {
        /** Use the LED Mode set in the current pipeline. */
        DEFAULT,
    
        /** Force off. */
        OFF,
    
        /** Force blink. */
        BLINK,
        
        /** Force on. */
        ON
    }

    private static class DoubleValue implements Supplier<Double> {
        private final NetworkTableEntry entry;
    
        DoubleValue(NetworkTable table, String key) {
            entry = table.getEntry(key);
        }

        /** Returns the current value, or NaN if there is no value. */
        @Override
        public Double get() {
            return entry.getDouble(Double.NaN);
        }
    }

    private static class DoubleArrayValue implements Supplier<Double[]> {
        private final NetworkTableEntry entry;
    
        DoubleArrayValue(NetworkTable table, String key) {
            entry = table.getEntry(key);
        }

        /** Returns the current value, or NaN if there is no value. */
        @Override
        public Double[] get() {
            return entry.getDoubleArray(new Double[0]);
        }
    }


    /** Whether the limelight has any valid targets. */
    public boolean hasTargets() {
        return hasTargets.get() == 1;
    }

    public double getTx() {
       return horizonalOffset.get();
    }

    public double getTy() {
        return verticalOffset.get();
    }

    public void setLedState(LedState state) {
        ledMode.setNumber(state.ordinal());
    }

    public LedState getLedState() {
        return LedState.values()[(int) ledMode.getNumber(0)];
    }

    public NetworkTableEntry getStream() {
        return stream;
    }

    /**
     * Gets the location on the field.<br>
     * If there location is undefined, returns an array with all 0s
     * @return a {@link Double} array with the following entries: x, y, z, pitch, yaw, roll
     * @see <a href="https://docs.limelightvision.io/en/latest/coordinate_systems_fiducials.html#field-space">Field space</a>
     */
    public Double[] getFieldLocation() {
        Double[] location = fieldLocation.get();
        if (location.length == 0) {
            location = new Double[6];
            for (int i = 0; i < location.length; i++) {
                location[i] = 0.0;
            }
        }
        return location;
    }
}
