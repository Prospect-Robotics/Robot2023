package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Drive;
import com.team2813.frc2023.util.Limelight;
import com.team2813.frc2023.util.NodeType;
import com.team2813.lib.util.LimelightValues.LedState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Consumer;

import static com.team2813.frc2023.Constants.*;

public class AutoAimCommand extends CommandBase {

    private final Limelight limelight;
    private final Drive driveSubsystem;
    private final Consumer<ChassisSpeeds> chassisSpeedsConsumer;
    private final NodeType nodeType;
    private final double txGoal = 0; // TODO: experimentally determine on comp bot

    private static final PIDController yController = new PIDController(0, 0, 0); // TODO: tune
    private static final PIDController thetaController = new PIDController(0, 0, 0); // TODO: tune
    private static final double MAX_SCALAR = 0.25;

    private double timeStart;
    private double timeDelta;
    private double currentHeading;
    private double txError;
    private double angularError;
    private double txDeadline;
    private double angularDeadline;
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

        if (nodeType == NodeType.CONE) {
            limelight.setPipeline(REFLECTIVE_TAPE_PIPELINE_INDEX);
            settingRequired = true;
        }
        if (!limelight.getValues().getLedState().equals(LedState.DEFAULT)) {
            limelight.setLights(true);
            settingRequired = true;
        }

        txError = txGoal - limelight.getValues().getTx();

        currentHeading = driveSubsystem.getRotation().getRadians() % (2 * Math.PI);
        angularError = 0 - currentHeading;
    }

    @Override
    public void execute() {
        timeDelta = Timer.getFPGATimestamp() - timeStart;
        if (!settingRequired || (timeDelta >= 0.125)) {
            double yScalar = MathUtil.clamp(
                    yController.calculate(limelight.getValues().getTx(), 0),
                    -MAX_SCALAR,
                    MAX_SCALAR
            );
            double yVelocity = -yScalar * Drive.MAX_VELOCITY;

            currentHeading = driveSubsystem.getRotation().getRadians() % (2 * Math.PI);
            double angularScalar = MathUtil.clamp(
                    thetaController.calculate(currentHeading, 0),
                    -MAX_SCALAR,
                    MAX_SCALAR
            );
            double angularVelocity = angularScalar * Drive.MAX_ANGULAR_VELOCITY;

            ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    0, yVelocity, angularVelocity, driveSubsystem.getRotation());

            chassisSpeedsConsumer.accept(targetChassisSpeeds);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // for testing purposes
    }

    @Override
    public void end(boolean interrupted) {
        chassisSpeedsConsumer.accept(new ChassisSpeeds());
    }
}
