package com.team2813.frc2023.commands;

import com.team2813.frc2023.commands.util.LockFunctionCommand;
import com.team2813.frc2023.subsystems.Arm;
import com.team2813.frc2023.subsystems.Pivot;
import com.team2813.frc2023.subsystems.Wrist;
import com.team2813.frc2023.util.NodeType;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.Supplier;

public class TopNodeConfigurationCommand extends SequentialCommandGroup {

    private static boolean firstRun = true;

    public TopNodeConfigurationCommand(Pivot pivotSubsystem, Arm armSubsystem, Wrist wristSubsystem, NodeType nodeType) {
        super(
                new ConditionalCommand(
                        new ZeroWristCommand(wristSubsystem),
                        new StowAllCommand(pivotSubsystem, armSubsystem, wristSubsystem),
                        () -> firstRun
                ),
                new InstantCommand(() -> firstRun = false),
                new LockFunctionCommand(pivotSubsystem::positionReached, () -> pivotSubsystem.setPosition(nodeType.getScoringPivotRotationsHigh()), pivotSubsystem),
                new ParallelCommandGroup(
                        new LockFunctionCommand(
                                armSubsystem::positionReached,
                                () -> armSubsystem.setPosition(Arm.ExtensionLength.TOP),
                                armSubsystem
                        ),
                        new LockFunctionCommand(
                                wristSubsystem::positionReached,
                                () -> wristSubsystem.setPosition(nodeType.getScoringWristRotationsHigh()),
                                wristSubsystem
                        )
                )
        );
    }

    public TopNodeConfigurationCommand(Pivot pivotSubsystem, Arm armSubsystem, Wrist wristSubsystem, Supplier<NodeType> nodeTypeSupplier) {
        super(
                new ConditionalCommand(
                        new ZeroWristCommand(wristSubsystem),
                        new StowAllCommand(pivotSubsystem, armSubsystem, wristSubsystem),
                        () -> firstRun
                ),
                new InstantCommand(() -> firstRun = false),
                new LockFunctionCommand(pivotSubsystem::positionReached, () -> pivotSubsystem.setPosition(nodeTypeSupplier.get().getScoringPivotRotationsHigh()), pivotSubsystem),
                new ParallelCommandGroup(
                        new LockFunctionCommand(
                                armSubsystem::positionReached,
                                () -> armSubsystem.setPosition(Arm.ExtensionLength.TOP),
                                armSubsystem
                        ),
                        new LockFunctionCommand(
                                wristSubsystem::positionReached,
                                () -> wristSubsystem.setPosition(nodeTypeSupplier.get().getScoringWristRotationsHigh()),
                                wristSubsystem
                        )
                )
        );
    }
}
