package com.team2813.frc2023.commands;

import com.team2813.frc2023.commands.util.LockFunctionCommand;
import com.team2813.frc2023.subsystems.Arm;
import com.team2813.frc2023.subsystems.Pivot;
import com.team2813.frc2023.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StowAllCommand extends ParallelCommandGroup {

    public StowAllCommand(Pivot pivotSubsystem, Arm armSubsystem, Wrist wristSubsystem) {
        super(
                new ZeroWristCommand(wristSubsystem),
                new SequentialCommandGroup(
                        new ZeroArmCommand(armSubsystem),
                        new ParallelCommandGroup(
                                new LockFunctionCommand(
                                        armSubsystem::positionReached,
                                        () -> armSubsystem.setPosition(Arm.ExtensionLength.INTAKE),
                                        armSubsystem
                                ),
                                new ZeroPivotCommand(pivotSubsystem)
                        )
                )
        );
    }
}
