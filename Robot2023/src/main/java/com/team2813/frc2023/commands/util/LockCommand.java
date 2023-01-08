package com.team2813.frc2023.commands.util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BooleanSupplier;

/**
 * Command that runs a function repeatedly until it returns true
 */
public class LockCommand extends CommandBase {

    private final BooleanSupplier lockFunction;
    private boolean unlocked = false;

    public LockCommand(BooleanSupplier lockFunction) {
        this.lockFunction = lockFunction;
    }

    public LockCommand(BooleanSupplier lockFunction, Subsystem... requirements) {
        this.lockFunction = lockFunction;
        addRequirements(requirements);
    }

    @Override
    public void execute() {
        try {
            unlocked = lockFunction.getAsBoolean();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public boolean isFinished() {
        return unlocked;
    }
}