package com.team2813.frc2023.commands;

import com.team2813.frc.util.Lightshow.Light;

import edu.wpi.first.wpilibj2.command.RepeatCommand;

public class BlinkLightsCommand extends RepeatCommand {

    public BlinkLightsCommand(Light light, double period){
        super(m_command)
    }
}