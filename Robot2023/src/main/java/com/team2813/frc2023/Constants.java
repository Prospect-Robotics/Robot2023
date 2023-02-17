// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2813.frc2023;

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
        public static final CommandPS4Controller AUTO_BALANCE_BUTTON = new JoystickButton(DRIVER_CONTROLLER, 1);

        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final CommandPS4Controller OPERATOR_CONTROLLER = new CommandPS4Controller(OPERATOR_CONTROLLER_PORT);
        public static final CommandPS4Controller TOP_NODE_BUTTON = new JoystickButton(OPERATOR_CONTROLLER, 1);
        public static final CommandPS4Controller MID_NODE_BUTTON = new JoystickButton(OPERATOR_CONTROLLER, 2);
        public static final CommandPS4Controller ZERO_ENCODER_BUTTON = new JoystickButton(OPERATOR_CONTROLLER, 3);
        public static final CommandPS4Controller TRIANGLE_INTAKE_BUTTON = new JoystickButton(OPERATOR_CONTROLLER, 4);
        public static final CommandPS4Controller RESET_WRIST_BUTTON = new JoystickButton(OPERATOR_CONTROLLER, 7);


        //Driver controls
        public static final Trigger SPATULA_TOGGLE_BUTTON = DRIVER_CONTROLLER.R1();
        public static final Trigger SLOW_MODE_BUTTON = DRIVER_CONTROLLER.L1();

        // Operator controls
        public static final Trigger INTAKE_BUTTON = OPERATOR_CONTROLLER.R1();
        public static final Trigger OUTTAKE_BUTTON = OPERATOR_CONTROLLER.L1();


        

        
        
    }
        public static int MOTOR_WRIST_ID = 17;

}
