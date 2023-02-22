package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Pivot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DefaultPivotCommand extends CommandBase {

    private final Pivot pivotSubsystem;
    private final DoubleSupplier additionalPosSupplier;

    public DefaultPivotCommand(DoubleSupplier additionalPosSupplier, Pivot armSubsystem) {
        this.additionalPosSupplier = additionalPosSupplier;
        this.pivotSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        double additionalPos = additionalPosSupplier.getAsDouble();
        if (Math.abs(additionalPos) > 0.2) {
            if (additionalPos > 0) {
                additionalPos = (additionalPos - 0.2) / (1 - 0.2);
            }
            else {
                additionalPos = (additionalPos + 0.2) / (1 - 0.2);
            }

            double position = pivotSubsystem.getMotorPosition() + (4 * additionalPos);
            position = MathUtil.clamp(position, 0, Pivot.Rotations.STARTING_CONFIGURATION.getPos());

            pivotSubsystem.setPosition(position);
        }
        else {
            pivotSubsystem.brake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.brake();
    }
}
