// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2813.frc2023;

import com.team2813.frc2023.Constants.OperatorConstants;
import com.team2813.frc2023.commands.Autos;
import com.team2813.frc2023.commands.DefaultDriveCommand;
import com.team2813.frc2023.commands.ExampleCommand;
import com.team2813.frc2023.subsystems.Drive;
import com.team2813.frc2023.subsystems.ExampleSubsystem;
import com.team2813.frc2023.util.Limelight;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.Map;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    /*
    String key is the name of the event marker in an auto routine,
    Command value is the command associated with that event marker.

    Refer to this when creating event markers in Path Planner.
    Note: this is just the default event map that is meant to have easy-to-
    predict commands (such as intake, place-cube-high, etc.), not commands such
    as rotating a specific number of degrees. You'll have to customize the event map
    that TrajectoryAutoBuilder.java uses to use commands like that.
     */
    public static final Map<String, Command> EVENT_MAP = new HashMap<>();

    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    private final Limelight limelight = Limelight.getInstance();
    private final Drive drive = new Drive();
    
    private final XboxController driver = new XboxController(0);
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        drive.setDefaultCommand(new DefaultDriveCommand(
                drive,
                () -> -modifyAxis(driver.getLeftY()) * Drive.MAX_VELOCITY,
                () -> -modifyAxis(driver.getLeftX()) * Drive.MAX_VELOCITY,
                () -> -modifyAxis(driver.getRightX()) * Drive.MAX_ANGULAR_VELOCITY
        ));

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
    private void configureBindings()
    {

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
}