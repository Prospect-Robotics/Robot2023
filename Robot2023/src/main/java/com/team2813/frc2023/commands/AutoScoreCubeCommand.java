package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoScoreCubeCommand extends SequentialCommandGroup {

    public AutoScoreCubeCommand(Intake intakeSubsystem) {
        super(
                new InstantCommand(intakeSubsystem::placeCube, intakeSubsystem),
                new WaitCommand(0.25),
                new InstantCommand(intakeSubsystem::stop, intakeSubsystem)
        );
    }
}
