package com.team2813.frc2023.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team2813.frc2023.subsystems.Arm;;

public class ZeroArmCommand extends CommandBase {
    private final Arm armSubsystem;

    private double startTime;

    public ZeroArmCommand(Arm armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.startRetractingArm();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) > 0.25 && armSubsystem.getMotorVelocity() == 0;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.brake();
        armSubsystem.zeroSensors();
    }
}
