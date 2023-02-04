package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LogCommand extends CommandBase {

    private final Drive driveSubsystem;

    public LogCommand(Drive driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        driveSubsystem.enableLogging(true);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.enableLogging(false);
    }
}
