package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Wrist;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroWristCommand extends CommandBase {

    private final Wrist wristSubsystem;

    private double startTime;

    public ZeroWristCommand(Wrist wristSubsystem) {
        this.wristSubsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        wristSubsystem.startStowingWrist();
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) > 0.25 && Math.abs(wristSubsystem.getMotorVelocity()) < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        wristSubsystem.idle();
        wristSubsystem.zeroSensors();
    }
}
