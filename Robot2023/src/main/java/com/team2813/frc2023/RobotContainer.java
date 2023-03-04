// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2813.frc2023;

import com.team2813.frc2023.commands.AutoSplineCommand.SubstationOffsetType;
import com.team2813.frc2023.commands.*;
import com.team2813.frc2023.commands.util.LockFunctionCommand;
import com.team2813.frc2023.subsystems.*;
import com.team2813.frc2023.util.NodeType;
import com.team2813.frc2023.util.ShuffleboardData;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.Map;

import static com.team2813.frc2023.Constants.OperatorConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Drive drive = new Drive();
    private final Spatula spatula = new Spatula();
    private final Pivot pivot = new Pivot();
    private final Arm arm = new Arm();
    private final Wrist wrist = new Wrist();
    private final Intake intake = new Intake();

    /**
     * String key is the name of the event marker in an auto routine,
     * Command value is the command associated with that event marker.
     * <p>
     * Refer to this when creating event markers in Path Planner.
     * Note: this is just the default event map that is meant to have easy-to-
     * predict commands, not commands such as rotating a specific number of degrees.
     * You'll have to customize the event map that TrajectoryAutoBuilder.java uses
     * to use commands like that
     * (use {@link com.team2813.frc2023.commands.util.TrajectoryAutoBuilder#customizeEventMap(Map)}).
     */
    public final Map<String, Command> EVENT_MAP = new HashMap<>() {{
        put("top-node-cone", new SequentialCommandGroup(
                new TopNodeConfigurationCommand(pivot, arm, wrist),
                new AutoScoreConeCommand(intake),
                new StowAllCommand(pivot, arm, wrist)
        ));
        put("top-node-cube", new SequentialCommandGroup(
                new TopNodeConfigurationCommand(pivot, arm, wrist),
                new AutoScoreCubeCommand(intake),
                new StowAllCommand(pivot, arm, wrist)
        ));
        put("mid-node-cone", new SequentialCommandGroup(
                new MidNodeConfigurationCommand(pivot, arm, wrist),
                new AutoScoreConeCommand(intake),
                new StowAllCommand(pivot, arm, wrist)
        ));
        put("mid-node-cube", new SequentialCommandGroup(
                new MidNodeConfigurationCommand(pivot, arm, wrist),
                new AutoScoreCubeCommand(intake),
                new StowAllCommand(pivot, arm, wrist)
        ));
    }};

    private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
    private final XboxController operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);

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

        // For spline testing purposes
        drive.initAutonomous(new Pose2d());

        // Configure the trigger bindings
        configureBindings();
    }

    Drive getDrive() {
        return drive;
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
    private void configureBindings() {
        Trigger apriltagSplineTrigger = new Trigger(() -> driverController.getLeftTriggerAxis() == 1);
        apriltagSplineTrigger.whileTrue(new AutoSplineCommand(apriltagSplineTrigger.negate(), NodeType.CUBE, drive));

        Trigger reflectiveSplineTrigger = new Trigger(() -> driverController.getRightTriggerAxis() == 1);
        reflectiveSplineTrigger.whileTrue(new AutoSplineCommand(reflectiveSplineTrigger.negate(), NodeType.CONE, drive));

        SLOWMODE_BUTTON.whileTrue(new InstantCommand(() -> drive.enableSlowMode(true), drive));
        SLOWMODE_BUTTON.onFalse(new InstantCommand(() -> drive.enableSlowMode(false), drive));
        SPATULA_BUTTON.toggleOnTrue(new StartEndCommand(spatula::extend, spatula::retract, spatula));

        TOP_NODE_BUTTON.onTrue(new TopNodeConfigurationCommand(pivot, arm, wrist));
        MID_NODE_BUTTON.onTrue(new MidNodeConfigurationCommand(pivot, arm, wrist));

        INTAKE_CUBE_BUTTON.whileTrue(new SequentialCommandGroup(
                new LockFunctionCommand(wrist::positionReached, () -> wrist.setPosition(Wrist.Rotations.INTAKE), wrist),
                new StartIntakeCommand(intake)
        ));
        INTAKE_CUBE_BUTTON.onFalse(new ParallelCommandGroup(
                new InstantCommand(intake::idle, intake),
                new ZeroWristCommand(wrist)
        ));

        Trigger intakeConeTrigger = new Trigger(() -> operatorController.getRightTriggerAxis() == 1);
        intakeConeTrigger.whileTrue(new SequentialCommandGroup(
                new LockFunctionCommand(wrist::positionReached, () -> wrist.setPosition(Wrist.Rotations.INTAKE), wrist),
                new StartIntakeCommand(intake)
        ));
        intakeConeTrigger.onFalse(new SequentialCommandGroup(
                new InstantCommand(intake::close, intake),
                new InstantCommand(intake::stop, intake),
                new WaitCommand(0.4),
                new ZeroWristCommand(wrist)
        ));

        SINGLE_SUB_BUTTON.whileTrue(new ParallelCommandGroup(
                new ZeroArmCommand(arm),
                new LockFunctionCommand(pivot::positionReached, () -> pivot.setPosition(Pivot.Rotations.SINGLE_SUBSTATION), pivot),
                new StartIntakeCommand(intake)
        ));
        SINGLE_SUB_BUTTON.onFalse(new SequentialCommandGroup(
                new InstantCommand(intake::close, intake),
                new InstantCommand(intake::stop, intake),
                new StowAllCommand(pivot, arm, wrist)
        ));

        Trigger doubleSubstationTrigger = new Trigger(() -> operatorController.getLeftTriggerAxis() == 1);
        doubleSubstationTrigger.whileTrue(new SequentialCommandGroup(
                new LockFunctionCommand(pivot::positionReached, () -> pivot.setPosition(Pivot.Rotations.HIGH), pivot),
                new ParallelCommandGroup(
                        new LockFunctionCommand(arm::positionReached, () -> arm.setPosition(Arm.ExtensionLength.DOUBLE_SUBSTATION), arm),
                        new LockFunctionCommand(wrist::positionReached, () -> wrist.setPosition(Wrist.Rotations.DOUBLE_SUBSTATION), wrist)
                ),
                new StartIntakeCommand(intake)
        ));
        doubleSubstationTrigger.onFalse(new SequentialCommandGroup(
                new InstantCommand(intake::close, intake),
                new InstantCommand(intake::stop, intake),
                new StowAllCommand(pivot, arm, wrist)
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
    public Command getAutonomousCommand() {
        AutoRoutine selectedRoutine = ShuffleboardData.routineChooser.getSelected();
        return selectedRoutine.getCommand();
    }

    public void populateMenus() {
        for (AutoRoutine routine : AutoRoutine.values()) {
            ShuffleboardData.routineChooser.addOption(routine.getName(), routine);
        }

        ShuffleboardData.offsetChooser.addOption("Left", SubstationOffsetType.LEFT);
        ShuffleboardData.offsetChooser.addOption("Right", SubstationOffsetType.RIGHT);
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
}