package com.team2813.frc2023.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team2813.frc2023.subsystems.Drive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static com.team2813.frc2023.Constants.*;

/**
 * Use before the first FollowCommand in an AutoRoutine.
 * If there are no FollowCommands, put this as the first
 * command in the AutoRoutine, using the last constructor.
 */
public class AutoInitDriveCommand extends InstantCommand {

    public AutoInitDriveCommand(String trajectoryName, Drive driveSubsystem) {
        super(() -> {
            PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryName, AUTO_MAX_VEL, AUTO_MAX_ACCEL);
            driveSubsystem.initAutonomous(trajectory.getInitialState());
        }, driveSubsystem);
    }

    public AutoInitDriveCommand(String trajectoryName, boolean reversed, Drive driveSubsystem) {
        super(() -> {
            PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryName, AUTO_MAX_VEL, AUTO_MAX_ACCEL, reversed);
            driveSubsystem.initAutonomous(trajectory.getInitialState());
        }, driveSubsystem);
    }

    public AutoInitDriveCommand(Rotation2d initialRotation, Drive driveSubsystem) {
        super(() -> driveSubsystem.initAutonomous(initialRotation), driveSubsystem);
    }
}