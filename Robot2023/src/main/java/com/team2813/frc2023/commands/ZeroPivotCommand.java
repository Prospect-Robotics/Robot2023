package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Pivot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroPivotCommand extends CommandBase {

    private final Pivot pivotSubsystem;

    private double startTime;

    public ZeroPivotCommand(Pivot pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        pivotSubsystem.startZeroingPivot();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) > 0.25 && Math.abs(pivotSubsystem.getMotorVelocity()) < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.brake();
        pivotSubsystem.zeroSensors();
    }
}
