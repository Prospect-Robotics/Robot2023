package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class StopIntakeCommand extends ParallelCommandGroup {

    public StopIntakeCommand(Intake intakeSubsystem) {
        super(
                new InstantCommand(intakeSubsystem::close, intakeSubsystem),
                new InstantCommand(intakeSubsystem::stop, intakeSubsystem)
        );
    }
}
