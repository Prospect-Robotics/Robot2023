package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Drive;
import com.team2813.frc2023.util.Limelight;
import com.team2813.frc2023.util.NodeType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Consumer;

import static com.team2813.frc2023.Constants.*;

public class AutoAimCommand extends CommandBase {

    private final Limelight limelight;
    private final Drive driveSubsystem;
    private final Consumer<ChassisSpeeds> chassisSpeedsConsumer;
    private final NodeType nodeType;

    private static final ProfiledPIDController yController = new ProfiledPIDController(
            0.01,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    Drive.MAX_VELOCITY * 0.25,
                    Drive.MAX_VELOCITY * 0.25)
    ); // TODO: tune

    private double timeStart;
    private double timeDelta;
    private double txDeadline;
    private boolean settingRequired = false;

    public AutoAimCommand(NodeType nodeType, Drive driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.nodeType = nodeType;

        chassisSpeedsConsumer = driveSubsystem::drive;
        limelight = Limelight.getInstance();

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        timeStart = Timer.getFPGATimestamp();
//        if (nodeType == NodeType.CONE) {
//            if (limelight.getValues().getPipelineIndex() != REFLECTIVE_TAPE_PIPELINE_INDEX) {
//                limelight.setPipeline(REFLECTIVE_TAPE_PIPELINE_INDEX);
//                settingRequired = true;
//            }
//        }
//        if (!limelight.getValues().getLedState().equals(LedState.DEFAULT)) {
//            limelight.setLights(true);
//            settingRequired = true;
//        }
//
//        timeStart = Timer.getFPGATimestamp() + (settingRequired ? 0.125 : 0);
//
//        double absoluteTxError = Math.abs(limelight.getValues().getTx());
    }

    @Override
    public void execute() {
        timeDelta = Timer.getFPGATimestamp() - timeStart;
//        if (!settingRequired || (timeDelta >= 0.125)) {
//            double yScalar = MathUtil.clamp(
//                    yController.calculate(limelight.getValues().getTx(), 0),
//                    -MAX_SCALAR,
//                    MAX_SCALAR
//            );
//            double yVelocity = -yScalar * Drive.MAX_VELOCITY;
//
//            currentHeading = driveSubsystem.getRotation().getRadians() % (2 * Math.PI);
//            double angularScalar = MathUtil.clamp(
//                    thetaController.calculate(currentHeading, 0),
//                    -MAX_SCALAR,
//                    MAX_SCALAR
//            );
//            double angularVelocity = angularScalar * Drive.MAX_ANGULAR_VELOCITY;
//
//            ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//                    0, yVelocity, angularVelocity, driveSubsystem.getRotation());
//
//            chassisSpeedsConsumer.accept(targetChassisSpeeds);
//        }

//        double angularScalar = MathUtil.clamp(
//                thetaController.calculate(driveSubsystem.getRotation().getRadians(), angularSetpoint),
//                -MAX_SCALAR,
//                MAX_SCALAR
//        );
//        double angularVelocity = angularScalar * Drive.MAX_ANGULAR_VELOCITY;
//
//        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//                0, 0, angularVelocity, driveSubsystem.getRotation());
//
//        chassisSpeedsConsumer.accept(targetChassisSpeeds);

        double yVelocity = yController.calculate(limelight.getValues().getTx(), 0);

        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0, yVelocity, 0, driveSubsystem.getRotation());

        chassisSpeedsConsumer.accept(targetChassisSpeeds);
    }

    @Override
    public boolean isFinished() {
        return false; // for testing purposes
    }

    @Override
    public void end(boolean interrupted) {
        chassisSpeedsConsumer.accept(new ChassisSpeeds());
        if (limelight.getValues().getPipelineIndex() != APRILTAG_PIPELINE_INDEX) limelight.setPipeline(APRILTAG_PIPELINE_INDEX);
        //limelight.setLights(false);
    }
}