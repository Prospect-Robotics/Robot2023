// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2813.frc2023;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        public static final Trigger GROUND_INTAKE_BUTTON = OPERATOR_CONTROLLER.square(); // actually maps to cross
        public static final Trigger SINGLE_SUB_BUTTON = OPERATOR_CONTROLLER.L1();

        public static final Trigger TOP_NODE_BUTTON = OPERATOR_CONTROLLER.triangle();
        public static final Trigger MID_NODE_BUTTON = OPERATOR_CONTROLLER.cross(); // actually maps to circle

        public static final Trigger INTAKE_CUBE_BUTTON = OPERATOR_CONTROLLER.R1();
        public static final Trigger OUTTAKE_BUTTON = OPERATOR_CONTROLLER.L1();
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
    public static final int INTAKE_MASTER_ID = 15;
    public static final int INTAKE_FOLLOWER_ID = 16;

    // Wrist
    public static int WRIST_MOTOR_ID = 17;

    // Pivot
    public static final int PIVOT_MOTOR_ID = 18;

    // Arm
    public static final int ARM_MOTOR_ID = 19;

    // Pneumatics
    public static final int PCM_ID = 20;

    public static final int INTAKE_PISTON_CHANNEL = 0;
    public static final int SPATULA_PISTON_CHANNEL_ONE = 1;
    public static final int SPATULA_PISTON_CHANNEL_TWO = 2;

    // Physical Constants

    public static final double TRACKWIDTH = Units.inchesToMeters(20.75); // meters
    public static final double WHEELBASE = Units.inchesToMeters(22.75); // meters
}
