package com.team2813.frc2023;

import com.team2813.frc2023.commands.*;
import com.team2813.frc2023.util.NodeType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static com.team2813.frc2023.Robot.*;

public enum AutoRoutine {

//    L1_1CO_1CU("L1-1Co-1Cu", AUTO_FACTORY.buildPathGroupAuto("L1 - 1Co - 1Cu")),
//    L1_1CO_INTAKE("L1-1Co-intake", AUTO_FACTORY.buildTrajectoryAuto("L1 - 1Co - intake")),
    SCORE_CONE("Score Cone", new SequentialCommandGroup(
            new InstantCommand(() -> ROBOT_CONTAINER.getDrive().initAutonomous(new Rotation2d(Math.PI))),
            new TopNodeConfigurationCommand(
                    ROBOT_CONTAINER.getPivot(),
                    ROBOT_CONTAINER.getArm(),
                    ROBOT_CONTAINER.getWrist(),
                    NodeType.CONE
            ),
            new AutoScoreConeCommand(ROBOT_CONTAINER.getIntake()),
            new StowAllCommand(
                    ROBOT_CONTAINER.getPivot(),
                    ROBOT_CONTAINER.getArm(),
                    ROBOT_CONTAINER.getWrist()
            )
    )),
    SCORE_CUBE("Score Cube", new SequentialCommandGroup(
            new InstantCommand(() -> ROBOT_CONTAINER.getDrive().initAutonomous(new Rotation2d(Math.PI))),
            new TopNodeConfigurationCommand(
                    ROBOT_CONTAINER.getPivot(),
                    ROBOT_CONTAINER.getArm(),
                    ROBOT_CONTAINER.getWrist(),
                    NodeType.CUBE
            ),
            new AutoScoreCubeCommand(ROBOT_CONTAINER.getIntake()),
            new StowAllCommand(
                    ROBOT_CONTAINER.getPivot(),
                    ROBOT_CONTAINER.getArm(),
                    ROBOT_CONTAINER.getWrist()
            )
    )),
    L1_1CO_MOBILITY("L1-1Cone-Mobility", AUTO_FACTORY.buildTrajectoryAuto("L1 - 1Co - Mobility")),
    L1_1CU_MOBILITY("L1-1Cube-Mobility", AUTO_FACTORY.buildTrajectoryAuto("L1 - 1Cu - Mobility")),
    L1_1CO_MOBILITY_CLIMB("L1-1Cone-Mobility-Climb", AUTO_FACTORY.buildTrajectoryAuto("L1 - 1Co - Mobility - Climb")),
    L1_1CU_MOBILITY_CLIMB("L1-1Cube-Mobility-Climb", AUTO_FACTORY.buildTrajectoryAuto("L1 - 1Cu - Mobility - Climb")),
    L1_2CU("L1-2Cube", AUTO_FACTORY.buildPathGroupAuto("L1 - 2Cu")),
//    L2_1CO_INTAKE1("L2-1Co-Intake1", AUTO_FACTORY.buildPathGroupAuto("L2 - 1Co - Intake1")),
//    L3_1CO_INTAKE("L3-1Co-intake", AUTO_FACTORY.buildPathGroupAuto("L3 - 1Co - intake")),
    L3_1CO_MOBILITY("L3-1Cone-Mobility", AUTO_FACTORY.buildTrajectoryAuto("L3 - 1Co - Mobility")),
    L3_1CU_MOBILITY("L3-1Cube-Mobility", AUTO_FACTORY.buildTrajectoryAuto("L3 - 1Cu - Mobility")),
    L3_1CO_MOBILITY_CLIMB("L3-1Cone-Mobility-Climb", AUTO_FACTORY.buildTrajectoryAuto("L3 - 1Co - Mobility - Climb")),
    L3_1CU_MOBILITY_CLIMB("L3-1Cube-Mobility-Climb", AUTO_FACTORY.buildTrajectoryAuto("L3 - 1Cu - Mobility - Climb")),
    L1_MOBILITY("L1-Mobility", AUTO_FACTORY.buildTrajectoryAuto("L1 - Mobility")),
    L1_GET_AWAY("L1-Get out of the way", AUTO_FACTORY.buildTrajectoryAuto("L1 - Get out of the way")),
    L1_MOBILITY_GET_AWAY("L1-Mobility-Get out of the way", AUTO_FACTORY.buildTrajectoryAuto("L1 - Mobility - Get out of the way")),
    L3_MOBILITY("L3-Mobility", AUTO_FACTORY.buildTrajectoryAuto("L3 - Mobility"));
//    AUTO_BALANCE_BLUE("Auto Balance Blue", new SequentialCommandGroup(
//            new InstantCommand(() -> ROBOT_CONTAINER.getDrive().initAutonomous(new Pose2d(14.68, 2.75, new Rotation2d(180)))),
//            new AutoBalanceCommand(ROBOT_CONTAINER.getDrive())
//    )),
//    AUTO_BALANCE_RED("Auto Balance Red", new SequentialCommandGroup(
//            new InstantCommand(() -> ROBOT_CONTAINER.getDrive().initAutonomous(new Pose2d(1.85, 2.75, new Rotation2d(180)))),
//            new AutoBalanceCommand(ROBOT_CONTAINER.getDrive())
//    )),
//    FORWARD_TEST("Forward Test", AUTO_FACTORY.buildTrajectoryAuto("Forward Test")),
//    STRAFE_TEST("Strafe Test", AUTO_FACTORY.buildTrajectoryAuto("Strafe Test")),
//    COMBO_TEST("Combo Test", AUTO_FACTORY.buildTrajectoryAuto("Combo Test"));
    
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
