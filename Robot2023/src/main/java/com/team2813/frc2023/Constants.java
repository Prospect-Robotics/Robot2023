// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2813.frc2023;

import edu.wpi.first.wpilibj.PS4Controller;
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
public final class Constants
{
    public static class OperatorConstants
    {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final CommandPS4Controller DRIVER_CONTROLLER = new CommandPS4Controller(DRIVER_CONTROLLER_PORT);

        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final CommandPS4Controller OPERATOR_CONTROLLER = new CommandPS4Controller(OPERATOR_CONTROLLER_PORT);
        

        // Operator controls
        public static final Trigger INTAKE_BUTTON = OPERATOR_CONTROLLER.R1();
        public static final Trigger OUTTAKE_BUTTON = OPERATOR_CONTROLLER.L1();

        public static final Trigger MID_NODE_POSITION = OPERATOR_CONTROLLER.circle();                                                                                       ;
        public static final Trigger TOP_NODE_POSITION = OPERATOR_CONTROLLER.triangle();
        public static final Trigger INTAKE_POSITION = OPERATOR_CONTROLLER.cross();

        public static final Trigger RESET_WRIST = OPERATOR_CONTROLLER.share();
        public static final Trigger RESET_ARM = OPERATOR_CONTROLLER.square();

    }


    // CAN IDs

    // Intake
    public static final int INTAKE_MASTER_ID = 15;
    public static final int INTAKE_FOLLOWER_ID = 16;

    // Arm
    public static final int ARM_MOTOR_ID = 18;

    // Pneumatics
    public static final int PCM_ID = 17;
    public static final int INTAKE_PISTON_CHANNEL = 0;
}
