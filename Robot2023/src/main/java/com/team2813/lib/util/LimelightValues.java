package com.team2813.lib.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightValues {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private NetworkTableEntry botposeEntry = table.getEntry("botepose");
    private NetworkTableEntry tv = table.getEntry("tv");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");
    private NetworkTableEntry ts = table.getEntry("ts");
    private NetworkTableEntry tl = table.getEntry("tl");

    public void update() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public NetworkTable getTable() {
        return table;
    }

    public NetworkTableEntry getBotposeEntry() {
        return botposeEntry;
    }

    public NetworkTableEntry getTv() {
        return tv;
    }

    public double getTx() {
        return tx.getDouble(0);
    }

    public double getTy() {
        return ty.getDouble(0);
    }

    public NetworkTableEntry getTa() {
        return ta;
    }

    public NetworkTableEntry getTs() {
        return ts;
    }

    public NetworkTableEntry getTl() {
        return tl;
    }

    public Number getX() {
        return botposeEntry.getNumberArray(new Number[]{0})[0];
    }

    public Number getY() {
        return botposeEntry.getNumberArray(new Number[]{0})[1];
    }
}
