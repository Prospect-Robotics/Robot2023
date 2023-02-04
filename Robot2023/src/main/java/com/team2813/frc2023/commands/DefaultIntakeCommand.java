package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DefaultIntakeCommand extends CommandBase {

    private final Intake intakeSubsystem;
    private final DoubleSupplier intakeTrigger;

    public DefaultIntakeCommand(DoubleSupplier intakeTrigger, Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakeTrigger = intakeTrigger;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (intakeTrigger.getAsDouble() == 1) intakeSubsystem.intake();
        else intakeSubsystem.stop();
    }
}
