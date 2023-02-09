// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2813.frc2023;

import com.team2813.frc2023.commands.Autos;
import com.team2813.frc2023.commands.DefaultArmCommand;
import com.team2813.frc2023.commands.ExampleCommand;
import com.team2813.frc2023.commands.ZeroArmCommand;
import com.team2813.frc2023.commands.util.LockFunctionCommand;
import com.team2813.frc2023.subsystems.Arm;
import com.team2813.frc2023.subsystems.ExampleSubsystem;
import com.team2813.frc2023.subsystems.Arm.ExtensionLength;
import com.team2813.frc2023.util.Limelight;
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
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    private final Limelight limelight = Limelight.getInstance();
    private final Arm arm = new Arm(); 


    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
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
        RESET_ARM.onTrue(new ZeroArmCommand(arm));
        MID_NODE_BUTTON.onTrue(new LockFunctionCommand(arm::positionReached, () -> arm.setPosition(ExtensionLength.MIDDLE)));
        INTAKE_POSITION_BUTTON.onTrue(new LockFunctionCommand(arm::positionReached, () -> arm.setPosition(ExtensionLength.INTAKE)));
        TOP_NODE_BUTTON.onTrue(new LockFunctionCommand(arm::positionReached, () -> arm.setPosition(ExtensionLength.TOP)));
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
}
