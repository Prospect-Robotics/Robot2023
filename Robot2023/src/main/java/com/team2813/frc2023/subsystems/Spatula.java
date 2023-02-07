package com.team2813.frc2023.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.team2813.lib.motors.SparkMaxWrapper;
import com.team2813.lib.solenoid.SolenoidGroup;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2813.frc2023.Constants.*;

public class Spatula extends SubsystemBase {

    private final SparkMaxWrapper motor = new SparkMaxWrapper(INTAKE_MASTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless, true);
    private final SolenoidGroup piston = new SolenoidGroup(PCM_ID, PneumaticsModuleType.CTREPCM, INTAKE_PISTON_CHANNEL);
    public void open() {
        piston.extend(); // actually retracts the piston
    }

    public void close() {
        piston.retract(); // actually extends the piston
    }

}
