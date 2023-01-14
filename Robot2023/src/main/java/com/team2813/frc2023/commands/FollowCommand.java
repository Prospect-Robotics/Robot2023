package com.team2813.frc2023.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.team2813.frc2023.subsystems.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Consumer;

import static com.team2813.frc2023.Constants.*;

/**
 * Command to follow a given trajectory
 */
public class FollowCommand extends PPSwerveControllerCommand {

    private static final PIDController xController = new PIDController(4, 0, 0);
    private static final PIDController yController = new PIDController(3.75, 0, 0);
    private static final PIDController thetaController = new PIDController(3.5, 1, 0);

    private final PathPlannerTrajectory trajectory;

    public FollowCommand(String trajectoryName, Drive driveSubsystem) {
        super(
                PathPlanner.loadPath(trajectoryName, AUTO_MAX_VEL, AUTO_MAX_ACCEL),
                driveSubsystem::getPose,
                xController,
                yController,
                thetaController,
                getChassisSpeedsConsumer(driveSubsystem),
                driveSubsystem
        );

        trajectory = PathPlanner.loadPath(trajectoryName, AUTO_MAX_VEL, AUTO_MAX_ACCEL);
    }

    public FollowCommand(String trajectoryName, boolean reversed, Drive driveSubsystem) {
        super(
                PathPlanner.loadPath(trajectoryName, AUTO_MAX_VEL, AUTO_MAX_ACCEL, reversed),
                driveSubsystem::getPose,
                xController,
                yController,
                thetaController,
                getChassisSpeedsConsumer(driveSubsystem),
                driveSubsystem
        );

        trajectory = PathPlanner.loadPath(trajectoryName, AUTO_MAX_VEL, AUTO_MAX_ACCEL, reversed);
    }

    private static Consumer<ChassisSpeeds> getChassisSpeedsConsumer(Drive driveSubsystem) {
        return new Consumer<ChassisSpeeds>() {
            @Override
            public void accept(ChassisSpeeds chassisSpeeds) {
                driveSubsystem.drive(chassisSpeeds);
            }
        };
    }

    @Override
    public void initialize() {
        super.initialize();

        PathPlannerState goalState = trajectory.getEndState();
        Pose2d goalPose = new Pose2d(goalState.poseMeters.getTranslation(), goalState.holonomicRotation);
        SmartDashboard.putString("Goal Pose", goalPose.toString());
    }
}