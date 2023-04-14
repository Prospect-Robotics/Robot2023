package com.team2813.frc2023.commands.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.team2813.frc2023.commands.FollowCommand;
import com.team2813.frc2023.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.Map;

import static com.team2813.frc2023.Constants.*;
import static com.team2813.frc2023.Robot.ROBOT_CONTAINER;

public class TrajectoryAutoBuilder extends BaseAutoBuilder {

    private final Drive driveSubsystem;

    public TrajectoryAutoBuilder(Drive driveSubsystem) {
        super(
                driveSubsystem::getPose,
                driveSubsystem::initAutonomous,
                ROBOT_CONTAINER.EVENT_MAP,
                DrivetrainType.HOLONOMIC,
                true
        );

        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public CommandBase followPath(PathPlannerTrajectory trajectory) {
        return new FollowCommand(trajectory, driveSubsystem);
    }

    public CommandBase buildTrajectoryAuto(String trajectoryName) {
        return buildTrajectoryAuto(trajectoryName, false);
    }

    public CommandBase buildTrajectoryAuto(String trajectoryName, boolean reversed) {
        return super.fullAuto(PathPlanner.loadPath(
                trajectoryName,
                new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL),
                reversed
        ));
    }

    public CommandBase buildPathGroupAuto(String pathGroupName) {
        return buildPathGroupAuto(pathGroupName, false);
    }

    // Use this if your trajectory has midway stop points
    public CommandBase buildPathGroupAuto(String pathGroupName, boolean reversed) {
        return super.fullAuto(PathPlanner.loadPathGroup(pathGroupName, AUTO_MAX_VEL, AUTO_MAX_ACCEL, reversed));
    }
}
