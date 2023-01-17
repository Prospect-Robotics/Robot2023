package com.team2813.frc2023.util;

import com.team2813.lib.util.LimelightValues;
import com.team2813.lib.util.LimelightValues.LedState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private final LimelightValues values = new LimelightValues();

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
        values.setLedState(enable ? LedState.DEFAULT : LedState.OFF);
    }

    public void setStream(int stream) {
        values.getStream().setNumber(stream);
    }

    /**
     * Gets the position on the playing field using build-in robot localization
     */
    // Do the output how you want to. (but make it make sense)
    public Pose2d getPosition() {
        Double[] location = values.getFieldLocation();
        location[0] = location[0] + 8.27;
        location[1] = location[1] + 4.01;
        Pose2d pose = new Pose2d(location[0], location[1], new Rotation2d(location[3], location[4]));
        return pose;
    }

    @Override
    public void periodic() {
        Pose2d position = getPosition();
        SmartDashboard.putNumber("tx", position.getX());
        SmartDashboard.putNumber("ty", position.getY());
        SmartDashboard.putBoolean("Valid apriltag", values.hasTargets());
        SmartDashboard.putNumber("Id of primary AprilTag", values.primaryApriltag());
    }
}
