package com.team2813.frc2023.commands;

import com.team2813.frc2023.commands.util.LockFunctionCommand;
import com.team2813.frc2023.subsystems.Arm;
import com.team2813.frc2023.subsystems.Pivot;
import com.team2813.frc2023.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MidNodeConfigurationCommand extends SequentialCommandGroup {

    private static boolean firstRun = true;

    public MidNodeConfigurationCommand(Pivot pivotSubsystem, Arm armSubsystem, Wrist wristSubsystem) {
        super(
                new ConditionalCommand(
                        new ZeroWristCommand(wristSubsystem),
                        new StowAllCommand(pivotSubsystem, armSubsystem, wristSubsystem),
                        () -> firstRun
                ),
                new InstantCommand(() -> firstRun = false),
                new LockFunctionCommand(pivotSubsystem::positionReached, () -> pivotSubsystem.setPosition(Pivot.Rotations.MID), pivotSubsystem),
                new ParallelCommandGroup(
                        new LockFunctionCommand(
                                armSubsystem::positionReached,
                                () -> armSubsystem.setPosition(Arm.ExtensionLength.MIDDLE),
                                armSubsystem
                        ),
                        new LockFunctionCommand(
                                wristSubsystem::positionReached,
                                () -> wristSubsystem.setPosition(Wrist.Rotations.MID_SCORE_CONE),
                                wristSubsystem
                        )
                )
        );
    }
}
