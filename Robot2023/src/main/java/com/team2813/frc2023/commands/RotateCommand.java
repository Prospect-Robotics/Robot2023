package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * Command to rotate by a given number of degrees.
 * Works like a unit circle:
 *      Positive # of degrees makes the robot turn counter-clockwise
 *      Negative # of degrees makes the robot turn clockwise
 */
public class RotateCommand extends CommandBase {

    private final Drive driveSubsystem;
    private final Consumer<ChassisSpeeds> chassisSpeedsConsumer;

    private double degreesToRotateBy;
    private DoubleSupplier degreeSupplier;
    private static final PIDController thetaController = new PIDController(.000025, 0, 0);

    private double setpoint;

    public RotateCommand(double degreesToRotateBy, Drive driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.degreesToRotateBy = degreesToRotateBy;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        chassisSpeedsConsumer = driveSubsystem::drive;
        addRequirements(driveSubsystem);
    }

    public RotateCommand(DoubleSupplier degreeSupplier, Drive driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.degreeSupplier = degreeSupplier;

        chassisSpeedsConsumer = driveSubsystem::drive;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        if (degreeSupplier != null) degreesToRotateBy = degreeSupplier.getAsDouble();
        setpoint = driveSubsystem.getRotation().getRadians() + Math.toRadians(degreesToRotateBy);
    }

    @Override
    public void execute() {
        double angularVelocity = thetaController.calculate(driveSubsystem.getRotation().getRadians(), setpoint);
        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0, 0, angularVelocity, driveSubsystem.getRotation());

        chassisSpeedsConsumer.accept(targetChassisSpeeds);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(setpoint - driveSubsystem.getRotation().getRadians()) <= Math.toRadians(1);
    }

    @Override
    public void end(boolean interrupted) {
        chassisSpeedsConsumer.accept(new ChassisSpeeds());
    }
}