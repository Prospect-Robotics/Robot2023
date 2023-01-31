package com.team2813.frc2023.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.team2813.frc2023.subsystems.Drive;
import com.team2813.frc2023.util.Limelight;
import com.team2813.frc2023.util.NodeType;
import com.team2813.frc2023.util.ShuffleboardData;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import static com.team2813.frc2023.Constants.*;

/**
 * Command that selects a node and then generates a spline
 * to that node and follows it. Node is selected based on
 * the angular offset from the center of the robot to the
 * node vision target. The node with the smallest angular
 * offset is picked.
 */
public class AutoSplineCommand extends SequentialCommandGroup {

    private static final Limelight limelight = Limelight.getInstance();
    private static final double NODE_OFFSET = Units.inchesToMeters(22);
    private static final double SUBSTATION_OFFSET = Units.inchesToMeters(35.25);

    private static double apriltagTx;
    private static int apriltagTxSign;
    private static Pose2d tagGoalPose;

    public static NodeType currentNodeType;

    public AutoSplineCommand(BooleanSupplier buttonLetGo, Drive driveSubsystem) {
        super(
                new InstantCommand(() -> limelight.setLights(true)),
                new WaitCommand(0.125),
                new InstantCommand(() -> {
                    Optional<Pose2d> robotPose = limelight.getPosition();
                    if (robotPose.isPresent()) {
                        driveSubsystem.resetOdometry(robotPose.get());

                        int apriltagID = limelight.getValues().primaryApriltag();
                        tagGoalPose = APRILTAG_MAP.get(apriltagID);

                        if ((apriltagID == 4) || (apriltagID == 5)) {
                            Pose2d goalPose = new Pose2d();
                            SubstationOffsetType offsetDirection = ShuffleboardData.offsetChooser.getSelected();

                            if (DriverStation.getAlliance().equals(Alliance.Red)) {
                                switch (offsetDirection) {
                                    case LEFT:
                                        goalPose = new Pose2d(
                                                tagGoalPose.getX(),
                                                tagGoalPose.getY() + SUBSTATION_OFFSET,
                                                tagGoalPose.getRotation()
                                        );
                                        break;
                                    case RIGHT:
                                        goalPose = new Pose2d(
                                                tagGoalPose.getX(),
                                                tagGoalPose.getY() - SUBSTATION_OFFSET,
                                                tagGoalPose.getRotation()
                                        );
                                        break;
                                }
                            }
                            else if (DriverStation.getAlliance().equals(Alliance.Blue)) {
                                switch (offsetDirection) {
                                    case LEFT:
                                        goalPose = new Pose2d(
                                                tagGoalPose.getX(),
                                                tagGoalPose.getY() - SUBSTATION_OFFSET,
                                                tagGoalPose.getRotation()
                                        );
                                        break;
                                    case RIGHT:
                                        goalPose = new Pose2d(
                                                tagGoalPose.getX(),
                                                tagGoalPose.getY() + SUBSTATION_OFFSET,
                                                tagGoalPose.getRotation()
                                        );
                                        break;
                                }
                            }

                            PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                                    new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL),
                                    List.of(
                                            PathPoint.fromCurrentHolonomicState(driveSubsystem.getPose(), driveSubsystem.getChassisSpeeds()),
                                            new PathPoint(
                                                    goalPose.getTranslation(),
                                                    goalPose.getRotation(),
                                                    goalPose.getRotation()
                                            )
                                    )
                            );

                            Command command = new ParallelRaceGroup(
                                    new WaitUntilCommand(buttonLetGo),
                                    new ParallelCommandGroup(
                                            new FollowCommand(trajectory, driveSubsystem),
                                            new InstantCommand(() -> limelight.setLights(false))
                                    )
                            );
                            command.schedule();
                        }
                        else {
                            apriltagTx = limelight.getValues().getTx();
                            apriltagTxSign = (int) (Math.abs(apriltagTx) / apriltagTx);

                            limelight.setPipeline(REFLECTIVE_TAPE_PIPELINE_INDEX);

                            Command command = new ParallelRaceGroup(
                                    new WaitUntilCommand(buttonLetGo),
                                    new SequentialCommandGroup(
                                            new DecideAndExecuteCommand(buttonLetGo, driveSubsystem),
                                            new InstantCommand(() -> limelight.setLights(false))
                                    )
                            );
                            command.schedule();
                        }
                    }
                    else {
                        limelight.setLights(false);
                    }
                }, driveSubsystem)
        );
    }

    private static class DecideAndExecuteCommand extends SequentialCommandGroup {

        private DecideAndExecuteCommand(BooleanSupplier buttonLetGo, Drive driveSubsystem) {
            super(
                    new WaitCommand(0.125),
                    new InstantCommand(() -> {
                        Pose2d goalPose = new Pose2d();

                        Optional<Double> horizontalOffsetOptional = limelight.getHorizontalOffset();
                        if (horizontalOffsetOptional.isPresent()) {
                            double tapeTx = horizontalOffsetOptional.get();
                            if (Math.abs(tapeTx) <= Math.abs(apriltagTx)) {
                                currentNodeType = NodeType.CONE;

                                int tapeTxSign = (int) (Math.abs(tapeTx) / tapeTx);
                                if (DriverStation.getAlliance().equals(Alliance.Red)) {
                                    if (tapeTxSign == apriltagTxSign) {
                                        if (tapeTxSign > 0) {
                                            goalPose = new Pose2d(
                                                    tagGoalPose.getX(),
                                                    tagGoalPose.getY() + NODE_OFFSET,
                                                    tagGoalPose.getRotation()
                                            );
                                        }
                                        else {
                                            goalPose = new Pose2d(
                                                    tagGoalPose.getX(),
                                                    tagGoalPose.getY() - NODE_OFFSET,
                                                    tagGoalPose.getRotation()
                                            );
                                        }
                                    }
                                    else {
                                        if (tapeTxSign > 0) {
                                            goalPose = new Pose2d(
                                                    tagGoalPose.getX(),
                                                    tagGoalPose.getY() - NODE_OFFSET,
                                                    tagGoalPose.getRotation()
                                            );
                                        }
                                        else {
                                            goalPose = new Pose2d(
                                                    tagGoalPose.getX(),
                                                    tagGoalPose.getY() + NODE_OFFSET,
                                                    tagGoalPose.getRotation()
                                            );
                                        }
                                    }
                                }
                                else if (DriverStation.getAlliance().equals(Alliance.Blue)) {
                                    if (tapeTxSign == apriltagTxSign) {
                                        if (tapeTxSign > 0) {
                                            goalPose = new Pose2d(
                                                    tagGoalPose.getX(),
                                                    tagGoalPose.getY() - NODE_OFFSET,
                                                    tagGoalPose.getRotation()
                                            );
                                        }
                                        else {
                                            goalPose = new Pose2d(
                                                    tagGoalPose.getX(),
                                                    tagGoalPose.getY() + NODE_OFFSET,
                                                    tagGoalPose.getRotation()
                                            );
                                        }
                                    }
                                    else {
                                        if (tapeTxSign > 0) {
                                            goalPose = new Pose2d(
                                                    tagGoalPose.getX(),
                                                    tagGoalPose.getY() + NODE_OFFSET,
                                                    tagGoalPose.getRotation()
                                            );
                                        }
                                        else {
                                            goalPose = new Pose2d(
                                                    tagGoalPose.getX(),
                                                    tagGoalPose.getY() - NODE_OFFSET,
                                                    tagGoalPose.getRotation()
                                            );
                                        }
                                    }
                                }
                            }
                            else {
                                currentNodeType = NodeType.CUBE;
                                goalPose = tagGoalPose;
                            }
                        }
                        else {
                            currentNodeType = NodeType.CUBE;
                            goalPose = tagGoalPose;
                        }

                        if (currentNodeType == NodeType.CUBE) limelight.setPipeline(APRILTAG_PIPELINE_INDEX);

                        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                                new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL),
                                List.of(
                                        PathPoint.fromCurrentHolonomicState(driveSubsystem.getPose(), driveSubsystem.getChassisSpeeds()),
                                        new PathPoint(
                                                goalPose.getTranslation(),
                                                goalPose.getRotation(),
                                                goalPose.getRotation()
                                        )
                                )
                        );

                        Command command = new ParallelRaceGroup(
                                new WaitUntilCommand(buttonLetGo),
                                new FollowCommand(trajectory, driveSubsystem)
                        );
                        command.schedule();
                    }, driveSubsystem)
            );
        }
    }

    public enum SubstationOffsetType {
        LEFT, RIGHT
    }
}
