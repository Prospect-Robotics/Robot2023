package com.team2813.lib.util;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;
import java.util.UUID;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightValuesTest {
    private NetworkTable table;
    private LimelightValues values;

    @BeforeEach
    void setNetworkTable() {
        String tableName = UUID.randomUUID().toString();
        table = NetworkTableInstance.getDefault().getTable(tableName);
        values = new LimelightValues(table);
    }

    @Test
    void getFieldLocation_withValidLocationInLimelight() {
        table.getEntry("botpose").setDoubleArray(new Double[] { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

        Double[] location = values.getFieldLocation();

        assertArrayEquals(new Double[] { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 }, location);
    }

    
    @Test
    void getFieldLocation_withoutValidLocationInLimelight() {
        Double[] location = values.getFieldLocation();

        assertArrayEquals(new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, location);
    }

    @Test
    void hasTargets_testSet() {
        table.getEntry("tv").setDouble(1.0);
        
        assertTrue(values.hasTargets(), "Failed for value=\"1.0\"");

        table.getEntry("tv").setDouble(0.0);

        assertFalse(values.hasTargets(), "Failed for value=\"0.0\"");
    }

    @Test
    void hasTargets_testNotSet() {
        assertEquals(false, values.hasTargets());
    }
}
