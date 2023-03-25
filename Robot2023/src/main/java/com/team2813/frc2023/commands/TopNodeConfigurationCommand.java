package com.team2813.frc2023.commands;

import com.team2813.frc2023.commands.util.LockFunctionCommand;
import com.team2813.frc2023.subsystems.Arm;
import com.team2813.frc2023.subsystems.Pivot;
import com.team2813.frc2023.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TopNodeConfigurationCommand extends SequentialCommandGroup {

    public TopNodeConfigurationCommand(Pivot pivotSubsystem, Arm armSubsystem, Wrist wristSubsystem) {
        super(
                new StowAllCommand(pivotSubsystem, armSubsystem, wristSubsystem),
                new LockFunctionCommand(pivotSubsystem::positionReached, () -> pivotSubsystem.setPosition(Pivot.Rotations.HIGH), pivotSubsystem),
                new ParallelCommandGroup(
                        new LockFunctionCommand(
                                armSubsystem::positionReached,
                                () -> armSubsystem.setPosition(Arm.ExtensionLength.TOP),
                                armSubsystem
                        ),
                        new LockFunctionCommand(
                                wristSubsystem::positionReached,
                                () -> wristSubsystem.setPosition(Wrist.Rotations.TOP_SCORE),
                                wristSubsystem
                        )
                )
        );
    }
}
