package com.team2813.frc2023.subsystems;

import com.team2813.lib.solenoid.SolenoidGroup;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2813.frc2023.Constants.*;

public class Spatula extends SubsystemBase {

    private final SolenoidGroup piston = new SolenoidGroup
            (PCM_ID, PneumaticsModuleType.CTREPCM, SPATULA_PISTON_CHANNEL_ONE, SPATULA_PISTON_CHANNEL_TWO);
    public void extend() {
        piston.extend();
    }

    public void retract() {
        piston.retract();
    }

}
