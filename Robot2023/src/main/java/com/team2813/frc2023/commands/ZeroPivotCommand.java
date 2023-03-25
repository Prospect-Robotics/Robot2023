package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Pivot;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroPivotCommand extends CommandBase {

    private final Pivot pivotSubsystem;

    public ZeroPivotCommand(Pivot pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        pivotSubsystem.startZeroingPivot();
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.atZero();
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.brake();
        pivotSubsystem.zeroSensors();
    }
}
