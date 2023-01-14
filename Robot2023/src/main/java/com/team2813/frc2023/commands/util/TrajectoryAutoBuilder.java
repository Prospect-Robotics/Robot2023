package com.team2813.frc2023.commands.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.team2813.frc2023.commands.FollowCommand;
import com.team2813.frc2023.subsystems.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Consumer;

import static com.team2813.frc2023.Constants.*;

public class TrajectoryAutoBuilder extends BaseAutoBuilder {

    private final Drive driveSubsystem;

    public TrajectoryAutoBuilder(Drive driveSubsystem) {
        super(
                driveSubsystem::getPose,
                new Consumer<Pose2d>() {
                    @Override
                    public void accept(Pose2d pose2d) {
                        driveSubsystem.initAutonomous(pose2d);
                    }
                },
                EVENT_MAP,
                DrivetrainType.HOLONOMIC
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

    public CommandBase buildPathGroupAuto(String pathGroupName, boolean reversed) {
        return super.fullAuto(PathPlanner.loadPathGroup(pathGroupName, AUTO_MAX_VEL, AUTO_MAX_ACCEL, reversed));
    }
}
