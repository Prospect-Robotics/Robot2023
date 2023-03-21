// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2813.frc2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.Map;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final CommandPS4Controller DRIVER_CONTROLLER = new CommandPS4Controller(DRIVER_CONTROLLER_PORT);

        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final CommandPS4Controller OPERATOR_CONTROLLER = new CommandPS4Controller(OPERATOR_CONTROLLER_PORT);

        // Driver controls
        public static final Trigger SLOWMODE_BUTTON = DRIVER_CONTROLLER.L1();
        public static final Trigger SPATULA_BUTTON = DRIVER_CONTROLLER.R1();

        // Operator controls
        public static final Trigger SINGLE_SUB_BUTTON = OPERATOR_CONTROLLER.L1();

        public static final Trigger TOP_NODE_BUTTON = OPERATOR_CONTROLLER.triangle();
        public static final Trigger MID_NODE_BUTTON = OPERATOR_CONTROLLER.cross(); // actually maps to circle

        public static final Trigger INTAKE_CUBE_BUTTON = OPERATOR_CONTROLLER.R1();
        public static final Trigger OUTTAKE_BUTTON = OPERATOR_CONTROLLER.circle(); // actually maps to square

        public static final Trigger WRIST_UP = OPERATOR_CONTROLLER.povUp();
        public static final Trigger WRIST_DOWN = OPERATOR_CONTROLLER.povDown();

        public static final Trigger STOW_BUTTON = OPERATOR_CONTROLLER.square(); // actually maps to cross
    }

    // IDs

    // Swerve Modules

    // Front Left
    public static final int FRONT_LEFT_DRIVE_ID = 2;
    public static final int FRONT_LEFT_STEER_ID = 3;
    public static final int FRONT_LEFT_ENCODER_ID = 4;


    // Front Right
    public static final int FRONT_RIGHT_DRIVE_ID = 5;
    public static final int FRONT_RIGHT_STEER_ID = 6;
    public static final int FRONT_RIGHT_ENCODER_ID = 7;

    // Back Left
    public static final int BACK_LEFT_DRIVE_ID = 8;
    public static final int BACK_LEFT_STEER_ID = 9;
    public static final int BACK_LEFT_ENCODER_ID = 10;

    // Back Right
    public static final int BACK_RIGHT_DRIVE_ID = 11;
    public static final int BACK_RIGHT_STEER_ID = 12;
    public static final int BACK_RIGHT_ENCODER_ID = 13;

    // Other Drive Stuff
    public static final int PIGEON_ID = 14;

    // Intake
    public static final int INTAKE_MOTOR_ID = 15;

    // Wrist
    public static int WRIST_MOTOR_ID = 17;

    // Pivot
    public static final int PIVOT_MOTOR_ID = 18;

    // Arm
    public static final int ARM_MOTOR_ID = 19;

    // LEDs
    public static final int CANIFIER_ID = 21;

    // Physical Constants
    public static final double TRACKWIDTH = Units.inchesToMeters(20.75); // meters
    public static final double WHEELBASE = Units.inchesToMeters(22.75); // meters

    // Auto Constants

    // Path Constraints
    public static final double AUTO_MAX_VEL = 4; // m/s
    public static final double AUTO_MAX_ACCEL = 5; // m/s^2

    // AprilTag ID location goals
    public static final Map<Integer, Pose2d> APRILTAG_MAP = new HashMap<>() {{
        put(1, new Pose2d(14.68, 1.06, new Rotation2d(0)));
        put(2, new Pose2d(14.68, 2.75, new Rotation2d(0)));
        put(3, new Pose2d(14.68, 4.43, new Rotation2d(0)));
        put(4, new Pose2d(15.64, 6.72, new Rotation2d(0)));
        put(5, new Pose2d(0.9, 6.72, new Rotation2d(Math.PI)));
        put(6, new Pose2d(1.85, 4.43, new Rotation2d(Math.PI)));
        put(7, new Pose2d(1.85, 2.75, new Rotation2d(Math.PI)));
        put(8, new Pose2d(1.85, 1.06, new Rotation2d(Math.PI)));
    }};

    // Limelight Pipeline Indices
    public static final int APRILTAG_PIPELINE_INDEX = 0;
    public static final int REFLECTIVE_TAPE_PIPELINE_INDEX = 1;
}
