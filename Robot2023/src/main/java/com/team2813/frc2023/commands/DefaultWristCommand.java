package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultWristCommand extends CommandBase {

    private final Wrist wristSubsystem;

    public DefaultWristCommand(Wrist wristSubsystem) {
        this.wristSubsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void execute() {
        wristSubsystem.setPosition(wristSubsystem.getMotorPosition());
    }
}
