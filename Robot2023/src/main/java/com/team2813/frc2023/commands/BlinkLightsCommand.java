package com.team2813.frc2023.commands;

import com.team2813.frc2023.util.Lightshow;
import com.team2813.frc2023.util.Lightshow.Light;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static com.team2813.frc2023.Robot.LIGHTSHOW;

public class BlinkLightsCommand extends RepeatCommand {

    public BlinkLightsCommand(Light light, double period) {
        super(new SequentialCommandGroup(
                new InstantCommand(() -> LIGHTSHOW.setLight(light)),
                new WaitCommand(period),
                new InstantCommand(() -> LIGHTSHOW.setLight(Light.DEFAULT)),
                new WaitCommand(period)
        ));
    }
}