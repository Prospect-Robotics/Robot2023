// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2813.frc2023;

import com.team2813.frc2023.commands.AutoSplineCommand;
import com.team2813.frc2023.commands.AutoSplineCommand.SubstationOffsetType;
import com.team2813.frc2023.commands.DefaultDriveCommand;
import com.team2813.frc2023.subsystems.Drive;
import com.team2813.frc2023.subsystems.Spatula;
import com.team2813.frc2023.util.Limelight;
import com.team2813.frc2023.util.NodeType;
import com.team2813.frc2023.util.ShuffleboardData;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
    private final Limelight limelight = Limelight.getInstance();

    /**
     * String key is the name of the event marker in an auto routine,
     * Command value is the command associated with that event marker.
     * <p>
     * Refer to this when creating event markers in Path Planner.
     * Note: this is just the default event map that is meant to have easy-to-
     * predict commands (such as intake, place-cube-high, etc.), not commands such
     * as rotating a specific number of degrees. You'll have to customize the event map
     * that TrajectoryAutoBuilder.java uses to use commands like that (use
     * {@link com.team2813.frc2023.commands.util.TrajectoryAutoBuilder#customizeEventMap(Map)}).
     */
    public final Map<String, Command> EVENT_MAP = new HashMap<>() {{
        
    }};

    private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drive.setDefaultCommand(new DefaultDriveCommand(
                () -> -modifyAxis(driverController.getLeftY()) * Drive.MAX_VELOCITY,
                () -> -modifyAxis(driverController.getLeftX()) * Drive.MAX_VELOCITY,
                () -> -modifyAxis(driverController.getRightX()) * Drive.MAX_ANGULAR_VELOCITY,
                drive
        ));

        // For spline testing purposes
        drive.initAutonomous(new Pose2d());

        // Configure the trigger bindings
        configureBindings();
    }

    Drive getDrive() {
        return drive;
    }

    Spatula getSpatula() {
        return spatula;
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