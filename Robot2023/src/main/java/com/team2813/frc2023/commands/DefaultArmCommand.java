package com.team2813.frc2023.commands;

import com.team2813.frc2023.subsystems.Arm;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DefaultArmCommand extends CommandBase {

    private final Arm armSubsystem;
    private final DoubleSupplier additionalPosSupplier;

    public DefaultArmCommand(DoubleSupplier additionalPosSupplier, Arm armSubsystem) {
        this.additionalPosSupplier = additionalPosSupplier;
        this.armSubsystem = armSubsystem;

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

            double position = armSubsystem.getMotorPosition() + additionalPos;
            position = MathUtil.clamp(position, 0, Arm.ExtensionLength.TOP.getPos());

            armSubsystem.setPosition(position);
        }
        else {
            armSubsystem.brake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.brake();
    }
}
