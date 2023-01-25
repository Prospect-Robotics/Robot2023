package com.team2813.frc2023.util;

import java.util.Optional;

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

    public void setPipeline(int pipelineIndex) {
        values.getPipelineIndex().setNumber(pipelineIndex);
    }

    /**
     * Gets the position on the playing field using build-in robot localization
     */
    // Do the output how you want to. (but make it make sense)
    public Optional<Pose2d> getPosition() {
        if (values.hasTargets()) {
            Double[] location = values.getFieldLocation();
            location[0] = location[0] + 8.27;
            location[1] = location[1] + 4.01;
            Pose2d pose = new Pose2d(location[0], location[1], Rotation2d.fromDegrees(location[5]));
            return Optional.of(pose);
        } else {
            return Optional.empty();
        }
    }

    public Optional<Double> getHorizontalOffset() {
        if (values.hasTargets()) return Optional.of(values.getTx());
        else return Optional.empty();
    }

    public Optional<Double> getVerticalOffset() {
        if (values.hasTargets()) return Optional.of(values.getTy());
        else return Optional.empty();
    }

    @Override
    public void periodic() {
        getPosition().ifPresent((Pose2d position) -> {
            SmartDashboard.putNumber("x position (meters):", position.getX());
            SmartDashboard.putNumber("y position (meters):", position.getY());
            SmartDashboard.putNumber("robot heading (degrees):", position.getRotation().getDegrees());
        });
        SmartDashboard.putBoolean("Valid apriltag", values.hasTargets());
        SmartDashboard.putNumber("Id of primary AprilTag", values.primaryApriltag());
    }
}
