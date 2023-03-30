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

//    private static final Limelight limelight = Limelight.getInstance();
//    private static final double NODE_OFFSET = Units.inchesToMeters(22);
//    private static final double SUBSTATION_OFFSET = Units.inchesToMeters(35.25);
//
//    private static Pose2d tagGoalPose;
//    private static double apriltagTx;
//
//    public AutoSplineCommand(BooleanSupplier spliningDisabled, NodeType nodeType, Drive driveSubsystem) {
//        super(
//                new InstantCommand(() -> limelight.setLights(true)),
//                //new WaitCommand(0.125),
//                new InstantCommand(() -> {
//                    Optional<Pose2d> robotPose = limelight.getPosition();
//                    if (robotPose.isPresent()) {
//                        driveSubsystem.resetOdometry(robotPose.get());
//
//                        int apriltagID = limelight.getValues().primaryApriltag();
//                        tagGoalPose = APRILTAG_MAP.get(apriltagID);
//
//                        if ((apriltagID == 4) || (apriltagID == 5)) {
//                            Pose2d goalPose = new Pose2d();
//                            SubstationOffsetType offsetDirection = ShuffleboardData.offsetChooser.getSelected();
//
//                            if (DriverStation.getAlliance().equals(Alliance.Red)) {
//                                switch (offsetDirection) {
//                                    case LEFT:
//                                        goalPose = new Pose2d(
//                                                tagGoalPose.getX(),
//                                                tagGoalPose.getY() + SUBSTATION_OFFSET,
//                                                tagGoalPose.getRotation()
//                                        );
//                                        break;
//                                    case RIGHT:
//                                        goalPose = new Pose2d(
//                                                tagGoalPose.getX(),
//                                                tagGoalPose.getY() - SUBSTATION_OFFSET,
//                                                tagGoalPose.getRotation()
//                                        );
//                                        break;
//                                }
//                            }
//                            else if (DriverStation.getAlliance().equals(Alliance.Blue)) {
//                                switch (offsetDirection) {
//                                    case LEFT:
//                                        goalPose = new Pose2d(
//                                                tagGoalPose.getX(),
//                                                tagGoalPose.getY() - SUBSTATION_OFFSET,
//                                                tagGoalPose.getRotation()
//                                        );
//                                        break;
//                                    case RIGHT:
//                                        goalPose = new Pose2d(
//                                                tagGoalPose.getX(),
//                                                tagGoalPose.getY() + SUBSTATION_OFFSET,
//                                                tagGoalPose.getRotation()
//                                        );
//                                        break;
//                                }
//                            }
//
//                            generateAndFollowSpline(spliningDisabled, goalPose, driveSubsystem);
//                        }
//                        else {
//                            switch (nodeType) {
//                                case CUBE:
//                                    generateAndFollowSpline(spliningDisabled, tagGoalPose, driveSubsystem);
//                                    break;
//                                case CONE:
//                                    apriltagTx = limelight.getValues().getTx();
//                                    limelight.setPipeline(REFLECTIVE_TAPE_PIPELINE_INDEX);
//
//                                    Command command = new ParallelRaceGroup(
//                                            new WaitUntilCommand(spliningDisabled),
//                                            new DecideAndExecuteCommand(spliningDisabled, driveSubsystem)
//                                    );
//                                    command.schedule();
//                            }
//                        }
//                    }
//                    else {
//                        limelight.setLights(false);
//                    }
//                }, driveSubsystem)
//        );
//    }
//
//    private static class DecideAndExecuteCommand extends SequentialCommandGroup {
//
//        private DecideAndExecuteCommand(BooleanSupplier spliningDisabled, Drive driveSubsystem) {
//            super(
//                    new WaitCommand(0.125),
//                    new InstantCommand(() -> {
//                        double tapeTx = limelight.getValues().getTx();
//                        Pose2d goalPose = new Pose2d();
//
//                        if (DriverStation.getAlliance().equals(Alliance.Red)) {
//                            if (tapeTx > apriltagTx) {
//                                goalPose = new Pose2d(
//                                        tagGoalPose.getX(),
//                                        tagGoalPose.getY() + NODE_OFFSET,
//                                        tagGoalPose.getRotation()
//                                );
//                            }
//                            else {
//                                goalPose = new Pose2d(
//                                        tagGoalPose.getX(),
//                                        tagGoalPose.getY() - NODE_OFFSET,
//                                        tagGoalPose.getRotation()
//                                );
//                            }
//                        }
//                        else if (DriverStation.getAlliance().equals(Alliance.Blue)) {
//                            if (tapeTx > apriltagTx) {
//                                goalPose = new Pose2d(
//                                        tagGoalPose.getX(),
//                                        tagGoalPose.getY() - NODE_OFFSET,
//                                        tagGoalPose.getRotation()
//                                );
//                            }
//                            else {
//                                goalPose = new Pose2d(
//                                        tagGoalPose.getX(),
//                                        tagGoalPose.getY() + NODE_OFFSET,
//                                        tagGoalPose.getRotation()
//                                );
//                            }
//                        }
//
//                        generateAndFollowSpline(spliningDisabled, goalPose, driveSubsystem);
//                    })
//            );
//        }
//    }
//
//    private static void generateAndFollowSpline(BooleanSupplier spliningDisabled, Pose2d goalPose, Drive driveSubsystem) {
//        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
//                new PathConstraints(AUTO_MAX_VEL, AUTO_MAX_ACCEL),
//                List.of(
//                        PathPoint.fromCurrentHolonomicState(driveSubsystem.getPose(), driveSubsystem.getChassisSpeeds()),
//                        new PathPoint(
//                                goalPose.getTranslation(),
//                                goalPose.getRotation(),
//                                goalPose.getRotation()
//                        )
//                )
//        );
//
//        Command command = new ParallelRaceGroup(
//                new WaitUntilCommand(spliningDisabled),
//                new ParallelCommandGroup(
//                        new FollowCommand(trajectory, driveSubsystem),
//                        new InstantCommand(() -> limelight.setLights(false)),
//                        new InstantCommand(() -> limelight.setPipeline(APRILTAG_PIPELINE_INDEX))
//                )
//        );
//        command.schedule();
//    }
//
//    public enum SubstationOffsetType {
//        LEFT, RIGHT
//    }
}
