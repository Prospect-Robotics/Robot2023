package com.team2813.frc2023.util;

import com.team2813.lib.util.LimelightValues;

public class Limelight {

    private LimelightValues values = new LimelightValues();

    private static final double MOUNT_ANGLE = 0; // degrees
    private static final double MOUNT_HEIGHT = 0; // inches
    private static final double TARGET_HEIGHT = 0; // inches

    private Limelight() {
        //
    }

    private static final Limelight instance = new Limelight();

    public static Limelight getInstance() {
        return instance;
    }

    public LimelightValues getValues() {
        return values;
    }

    public void setLights(boolean enable) {
        values.getLedMode().setNumber(enable ? 0 : 1);
    }

    public void setStream(int stream) {
        values.getStream().setNumber(stream);
    }

    /**
     * Gets the position on the playing field using position relative to Apriltags
     */
    // Do the output how you want to. (but make it make sense)
    public void getPositionTracking() {
        // TODO: add code here
    }

    /**
     * Gets the position on the playing field using build-in robot localization
     */
    // Do the output how you want to. (but make it make sense)
    public void getPositionLocalization() {
        // TODO: add code here
    }
}
