package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Pivot;
import edu.wpi.first.wpilibj.DriverStation;
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
        startTime = Timer.getFPGATimestamp();
        pivotSubsystem.startZeroingPivot();
    }

    @Override
    public boolean isFinished() {
        if (pivotSubsystem.atZero()) return true;
        else if (((Timer.getFPGATimestamp() - startTime) > 0.25 && Math.abs(pivotSubsystem.getMotorVelocity()) < 0.1)) {
            DriverStation.reportError("Limit switch not reached", false);
            return true;
        }
        else return false;
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.brake();
        pivotSubsystem.zeroSensors();
    }
}
