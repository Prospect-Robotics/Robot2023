// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2813.frc2023;

import com.team2813.frc2023.Constants.OperatorConstants;
import com.team2813.frc2023.commands.Autos;
import com.team2813.frc2023.commands.DefaultArmCommand;
import com.team2813.frc2023.commands.ExampleCommand;
import com.team2813.frc2023.subsystems.Arm;
import com.team2813.frc2023.subsystems.ExampleSubsystem;
import com.team2813.frc2023.util.Limelight;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    
    private final XboxController operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        arm.setDefaultCommand(new DefaultArmCommand(
                () -> -operatorController.getRightY(),
                arm
        ));

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
//        INTAKE_BUTTON.whileTrue(new SequentialCommandGroup(
//                new InstantCommand(intake::open, intake),
//                new WaitCommand(0.4),
//                new InstantCommand(intake::intake, intake)
//        ));
//        INTAKE_BUTTON.onFalse(new StopIntakeCommand(intake));
//
//        OUTTAKE_BUTTON.whileTrue(new SequentialCommandGroup(
//                new InstantCommand(intake::open, intake),
//                new WaitCommand(1),
//                new InstantCommand(intake::outtake, intake)
//        ));
//        OUTTAKE_BUTTON.onFalse(new StopIntakeCommand(intake));
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
