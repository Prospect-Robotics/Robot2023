package com.team2813.frc2023.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2813.lib.motors.ControlMode;
import com.team2813.lib.motors.SparkMaxWrapper;
import com.team2813.lib.solenoid.SolenoidGroup;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2813.frc2023.Constants.*;

public class Intake extends SubsystemBase {
    // Neo 550 and Falcon motors are brushless, kBrushless enum value - refer to MotorType class 
    private final SparkMaxWrapper motor = new SparkMaxWrapper(INTAKE_MASTER_ID, true, MotorType.kBrushless);
    private final SolenoidGroup piston = new SolenoidGroup(PCM_ID, PneumaticsModuleType.CTREPCM, INTAKE_PISTON_CHANNEL);

    public Intake () {
        motor.addFollower(INTAKE_FOLLOWER_ID, MotorType.kBrushless, true);
    }

    public void open() {
        piston.retract();
    }

    public void close() {
        piston.extend();
    }

    public void intake () {
        motor.set(ControlMode.DUTY_CYCLE, .5 ); 
    }

    public void outtake() {
        motor.set(ControlMode.DUTY_CYCLE, -.05 ); 
    }

    public void stop() {
        motor.set(ControlMode.DUTY_CYCLE, 0);
    }
}