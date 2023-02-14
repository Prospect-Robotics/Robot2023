package com.team2813.frc2023.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.team2813.frc2023.subsystems.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Consumer;

/**
 * Command to follow a given trajectory
 */
public class FollowCommand extends PPSwerveControllerCommand {

    private static final PIDController xController = new PIDController(1.5, 0, 0);
    private static final PIDController yController = new PIDController(2, 0, 0);
    private static final PIDController thetaController = new PIDController(2, 0, 0);

    private final Consumer<ChassisSpeeds> chassisSpeedsConsumer;

    public FollowCommand(PathPlannerTrajectory trajectory, Drive driveSubsystem) {
        super(
                trajectory,
                driveSubsystem::getPose,
                xController,
                yController,
                thetaController,
                driveSubsystem::drive,
                true,
                driveSubsystem
        );

        chassisSpeedsConsumer = driveSubsystem::drive;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        chassisSpeedsConsumer.accept(new ChassisSpeeds());
    }
}