package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StartIntakeCommand extends SequentialCommandGroup {

    public StartIntakeCommand(Intake intakeSubsystem) {
        super(
                new InstantCommand(intakeSubsystem::open, intakeSubsystem),
                new InstantCommand(intakeSubsystem::intake, intakeSubsystem)
        );
    }
}
