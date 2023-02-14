package com.team2813.frc2023;

import com.team2813.frc2023.commands.RotateCommand;
import edu.wpi.first.wpilibj2.command.Command;

import static com.team2813.frc2023.Robot.*;

public enum AutoRoutine {

    // Commented out because they are not ready yet, will crash the code
//    L1_1CO_1CU("L1-1Co-1Cu", AUTO_FACTORY.buildPathGroupAuto("L1 - 1Co - 1Cu")),
//    L1_1CO_CLIMB("L1-1Co-Climb", AUTO_FACTORY.buildTrajectoryAuto("L1 - 1Co - Climb")),
//    L2_1CO_INTAKE1("L2-1Co-Intake1", AUTO_FACTORY.buildPathGroupAuto("L2 - 1Co - Intake1")),
//    L3_1CO_1CU("L3-1Co-1Cu", AUTO_FACTORY.buildPathGroupAuto("L3 - 1Co - 1Cu")),
//    L3_1CO_CLIMB("L3-1Co-Climb", AUTO_FACTORY.buildTrajectoryAuto("L3 - 1Co - Climb")),
    L1_3CO_KNOCK2("L1-3Co-Knock2", AUTO_FACTORY.buildPathGroupAuto("L1 - 3Co - Knock 2")),
    L1_1C_GET_AWAY("L1-1C-Get out of the way", AUTO_FACTORY.buildTrajectoryAuto("L1 - 1C - Get out of the way")),
    L3_MOBILITY("L3-Mobility", AUTO_FACTORY.buildTrajectoryAuto("L3 - Mobility")),
    L1_MOBILITY_GET_AWAY("L1-Mobility-Get out of the way", AUTO_FACTORY.buildTrajectoryAuto("L1 - Mobility - Get out of the way")),
    FORWARD_TEST("Forward Test", AUTO_FACTORY.buildTrajectoryAuto("Forward Test")),
    STRAFE_TEST("Strafe Test", AUTO_FACTORY.buildTrajectoryAuto("Strafe Test")),
    ROTATE_TEST("Rotate Test", new RotateCommand(90, ROBOT_CONTAINER.getDrive()));
    
    private final String name;
    private final Command command;

    AutoRoutine(String name, Command command) {
        this.name = name;
        this.command = command;
    }

    public String getName() {
        return name;
    }

    public Command getCommand() {
        return command;
    }
}
