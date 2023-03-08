package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoScoreConeCommand extends SequentialCommandGroup {

    public AutoScoreConeCommand(Intake intakeSubsystem) {
        super(
                new InstantCommand(intakeSubsystem::open, intakeSubsystem),
                new WaitCommand(0.25),
                new InstantCommand(intakeSubsystem::close, intakeSubsystem)
        );
    }
}
