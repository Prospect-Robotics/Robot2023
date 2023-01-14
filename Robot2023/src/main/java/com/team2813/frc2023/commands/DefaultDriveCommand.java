package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Drive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final Drive driveSubsystem;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public DefaultDriveCommand(Drive driveSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(),
                rotationSupplier.getAsDouble(),
                driveSubsystem.getRotation()
        ));
    }
}
