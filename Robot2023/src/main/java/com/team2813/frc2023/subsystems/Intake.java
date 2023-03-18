package com.team2813.frc2023.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2813.lib.motors.ControlMode;
import com.team2813.lib.motors.SparkMaxWrapper;
import com.team2813.lib.solenoid.SolenoidGroup;
import com.team2813.lib.util.ConfigUtils;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2813.frc2023.Constants.*;

public class Intake extends SubsystemBase {
    // Neo 550 and Falcon motors are brushless, kBrushless enum value - refer to MotorType class 
    private final SparkMaxWrapper motorOne = new SparkMaxWrapper(INTAKE_MASTER_ID, MotorType.kBrushless, false);
    private final SparkMaxWrapper motorTwo = new SparkMaxWrapper(INTAKE_FOLLOWER_ID, MotorType.kBrushless, true);

    public Intake() {
        ConfigUtils.revConfig(() -> motorOne.setSmartCurrentLimit(30));
        ConfigUtils.revConfig(() -> motorTwo.setSmartCurrentLimit(30));
    }

    public void intake () {
        motorOne.set(ControlMode.DUTY_CYCLE, .6);
        motorTwo.set(ControlMode.DUTY_CYCLE, .6);
    }

    public void outtake() {
        motorOne.set(ControlMode.DUTY_CYCLE, -.2);
        motorTwo.set(ControlMode.DUTY_CYCLE, -.2);
    }

    public void idle() {
        motorOne.set(ControlMode.DUTY_CYCLE, .05);
        motorTwo.set(ControlMode.DUTY_CYCLE, .05);
    }

    public void stop() {
        motorOne.set(ControlMode.DUTY_CYCLE, 0);
        motorTwo.set(ControlMode.DUTY_CYCLE, 0);
    }
}
