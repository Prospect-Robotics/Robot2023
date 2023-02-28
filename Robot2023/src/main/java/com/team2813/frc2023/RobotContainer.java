// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2813.frc2023;

import com.team2813.frc2023.commands.*;
import com.team2813.frc2023.commands.util.LockFunctionCommand;
import com.team2813.frc2023.subsystems.*;
import com.team2813.frc2023.util.Limelight;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.team2813.frc2023.Constants.OperatorConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final Drive drive = new Drive();
    private final Spatula spatula = new Spatula();
    private final Pivot pivot = new Pivot();
    private final Arm arm = new Arm();
    private final Wrist wrist = new Wrist();
    private final Intake intake = new Intake();
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    private final Limelight limelight = Limelight.getInstance();
    
    private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
    private final XboxController operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);

    private IntakeType currentIntakeMode = IntakeType.GROUND;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        drive.setDefaultCommand(new DefaultDriveCommand(
                () -> -modifyAxis(driverController.getLeftY()) * Drive.MAX_VELOCITY,
                () -> -modifyAxis(driverController.getLeftX()) * Drive.MAX_VELOCITY,
                () -> -modifyAxis(driverController.getRightX()) * Drive.MAX_ANGULAR_VELOCITY,
                drive
        ));

        pivot.setDefaultCommand(new DefaultPivotCommand(() -> -operatorController.getLeftY(), pivot));
        arm.setDefaultCommand(new DefaultArmCommand(() -> -operatorController.getRightY(), arm));
        wrist.setDefaultCommand(new DefaultWristCommand(wrist));

        // Configure the trigger bindings
        configureBindings();
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        SLOWMODE_BUTTON.whileTrue(new InstantCommand(() -> drive.enableSlowMode(true), drive));
        SLOWMODE_BUTTON.onFalse(new InstantCommand(() -> drive.enableSlowMode(false), drive));
        SPATULA_BUTTON.toggleOnTrue(new StartEndCommand(spatula::extend, spatula::retract, spatula));

        TOP_NODE_BUTTON.onTrue(new SequentialCommandGroup(
                new LockFunctionCommand(pivot::positionReached, () -> pivot.setPosition(Pivot.Rotations.HIGH), pivot),
                new ParallelCommandGroup(
                        new LockFunctionCommand(arm::positionReached, () -> arm.setPosition(Arm.ExtensionLength.TOP), arm),
                        new LockFunctionCommand(wrist::positionReached, () -> wrist.setPosition(Wrist.Rotations.OUTTAKE), wrist)
                )
        ));

        MID_NODE_BUTTON.onTrue(new SequentialCommandGroup(
                new LockFunctionCommand(pivot::positionReached, () -> pivot.setPosition(Pivot.Rotations.MID), pivot),
                new ParallelCommandGroup(
                        new LockFunctionCommand(arm::positionReached, () -> arm.setPosition(Arm.ExtensionLength.MIDDLE), arm),
                        new LockFunctionCommand(wrist::positionReached, () -> wrist.setPosition(Wrist.Rotations.OUTTAKE), wrist)
                )
        ));

        INTAKE_CUBE_BUTTON.whileTrue(new StartIntakeCommand(intake));
        INTAKE_CUBE_BUTTON.onFalse(new ParallelCommandGroup(
                new InstantCommand(intake::idle, intake),
                new InstantCommand(() -> {
                    if (currentIntakeMode.equals(IntakeType.GROUND)) new ZeroWristCommand(wrist).schedule();
                    else new StowAllCommand(pivot, arm, wrist).schedule();
                })
        ));

        Trigger intakeConeTrigger = new Trigger(() -> operatorController.getRightTriggerAxis() == 1);
        intakeConeTrigger.whileTrue(new SequentialCommandGroup(
                new InstantCommand(intake::open, intake),
                new InstantCommand(intake::intake, intake)
        ));
        intakeConeTrigger.onFalse(new SequentialCommandGroup(
                new InstantCommand(intake::close, intake),
                new InstantCommand(intake::stop, intake),
                new WaitCommand(0.4),
                new InstantCommand(() -> {
                    if (currentIntakeMode.equals(IntakeType.GROUND)) new ZeroWristCommand(wrist).schedule();
                    else new StowAllCommand(pivot, arm, wrist).schedule();
                })
        ));

        GROUND_INTAKE_BUTTON.onTrue(new SequentialCommandGroup(
                new LockFunctionCommand(wrist::positionReached, () -> wrist.setPosition(Wrist.Rotations.INTAKE), wrist),
                new InstantCommand(() -> currentIntakeMode = IntakeType.GROUND)
        ));

        SINGLE_SUB_BUTTON.onTrue(new ParallelCommandGroup(
                new ZeroArmCommand(arm),
                new LockFunctionCommand(pivot::positionReached, () -> pivot.setPosition(Pivot.Rotations.SINGLE_SUBSTATION), pivot),
                new InstantCommand(() -> currentIntakeMode = IntakeType.SINGLE_SUB)
        ));

        Trigger doubleSubstationTrigger = new Trigger(() -> operatorController.getLeftTriggerAxis() == 1);
        doubleSubstationTrigger.onTrue(new SequentialCommandGroup(
                new LockFunctionCommand(pivot::positionReached, () -> pivot.setPosition(Pivot.Rotations.HIGH), pivot),
                new ParallelCommandGroup(
                        new LockFunctionCommand(arm::positionReached, () -> arm.setPosition(Arm.ExtensionLength.DOUBLE_SUBSTATION), arm),
                        new LockFunctionCommand(wrist::positionReached, () -> wrist.setPosition(Wrist.Rotations.DOUBLE_SUBSTATION), wrist)
                ),
                new InstantCommand(() -> currentIntakeMode = IntakeType.DOUBLE_SUB)
        ));

        OUTTAKE_BUTTON.whileTrue(new SequentialCommandGroup(
                new InstantCommand(intake::open, intake),
                new WaitCommand(0.25),
                new InstantCommand(intake::outtake, intake)
        ));
        OUTTAKE_BUTTON.onFalse(new SequentialCommandGroup(
                new InstantCommand(intake::stop, intake),
                new InstantCommand(intake::close, intake),
                new StowAllCommand(pivot, arm, wrist)
        ));

        WRIST_UP.whileTrue(new InstantCommand(wrist::up, wrist));
        WRIST_UP.onFalse(new InstantCommand(wrist::brake, wrist));

        WRIST_DOWN.whileTrue(new InstantCommand(wrist::down, wrist));
        WRIST_DOWN.onFalse(new InstantCommand(wrist::brake, wrist));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return Autos.exampleAuto(exampleSubsystem);
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0) {
                return (value - deadband) / (1 - deadband);
            } else {
                return (value + deadband) / (1 - deadband);
            }
        } else {
            return 0;
        }
    }

    private static double modifyAxis(double value) {
        value = deadband(value, 0.1);
        value = Math.copySign(value * value, value);
        return value;
    }

    private enum IntakeType {
        GROUND, SINGLE_SUB, DOUBLE_SUB
    }
}
