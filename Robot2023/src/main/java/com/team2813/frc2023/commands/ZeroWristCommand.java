package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroWristCommand extends CommandBase {

    private final Wrist wristSubsystem;

    public ZeroWristCommand(Wrist wristSubsystem) {
        this.wristSubsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        wristSubsystem.startStowingWrist();
    }

    @Override
    public boolean isFinished() {
        return wristSubsystem.atZero();
    }

    @Override
    public void end(boolean interrupted) {
        wristSubsystem.brake();
        if (!interrupted) wristSubsystem.zeroSensors();
    }
}
