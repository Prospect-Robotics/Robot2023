package com.team2813.frc2023.util;

import com.team2813.frc2023.AutoRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleboardData {
    // define your smartdashboard entries here
    public static SendableChooser<AutoRoutine> routineChooser = new SendableChooser<>();

    public static void init() {
        SmartDashboard.putData("Auto Routine", routineChooser);
    }
}
