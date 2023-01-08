package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Drive;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Consumer;

/**
 * Command to rotate by a given number of degrees.
 * Works like a unit circle:
 *      Positive # of degrees makes the robot turn counter-clockwise
 *      Negative # of degrees makes the robot turn clockwise
 */
public class RotateCommand extends CommandBase {

    private final Drive driveSubsystem;
    private final Consumer<SwerveModuleState[]> swerveModuleStatesConsumer;
    private final double degreesToRotateBy;

    private static final ProfiledPIDController thetaController = new ProfiledPIDController(
            1,
            0,
            0.0225,
            new TrapezoidProfile.Constraints(Drive.MAX_ANGULAR_VELOCITY, Drive.MAX_ANGULAR_ACCELERATION)
    );

    private double setpoint;

    public RotateCommand(double degreesToRotateBy, Drive driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.degreesToRotateBy = degreesToRotateBy;

        swerveModuleStatesConsumer = getSwerveModuleStatesConsumer(driveSubsystem);
        addRequirements(driveSubsystem);
    }

    private static Consumer<SwerveModuleState[]> getSwerveModuleStatesConsumer(Drive driveSubsystem) {
        return new Consumer<SwerveModuleState[]>() {
            @Override
            public void accept(SwerveModuleState[] swerveModuleStates) {
                driveSubsystem.drive(swerveModuleStates);
            }
        };
    }

    @Override
    public void initialize() {
        thetaController.reset(driveSubsystem.getRotation().getRadians());
        setpoint = driveSubsystem.getRotation().getRadians() + Math.toRadians(degreesToRotateBy);
    }

    @Override
    public void execute() {
        double angularVelocity = thetaController.calculate(driveSubsystem.getRotation().getRadians(), setpoint);
        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0, 0, angularVelocity, driveSubsystem.getRotation());
        SwerveModuleState[] targetModuleStates = driveSubsystem.getKinematics().toSwerveModuleStates(targetChassisSpeeds);

        swerveModuleStatesConsumer.accept(targetModuleStates);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(setpoint - driveSubsystem.getRotation().getRadians()) <= Math.toRadians(2.5);
    }

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleState[] targetModuleStates = driveSubsystem.getKinematics().toSwerveModuleStates(targetChassisSpeeds);

        swerveModuleStatesConsumer.accept(targetModuleStates);
    }
}