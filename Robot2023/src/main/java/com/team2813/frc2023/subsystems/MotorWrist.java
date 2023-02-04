package com.team2813.frc2023.subsystems;

import com.team2813.lib.motors.SparkMaxWrapper;
import com.team2813.lib.motors.ControlMode;
import static com.team2813.frc2023.Constants.*;
//TODO: Find import for MotorType


public class MotorWrist extends SOMETHING_PROBABLY_SUBSYSTEM_1D {
    public MotorWrist() {
        super(new SparkMaxWrapper(MOTOR_WRIST_ID, MotorType.kBrushless, false)); //TODO: Find ot if its inverted or not
        motor.configPID(0, 0, 0) // TODO: Tune PID
    }

    public double getMotorVelocity() {
        return motor.getVelocity();
    }

    public void startRotatingWrist() {
        motor.set(ControlMode.DUTY_CYCLE, ??) // TODO: Find value to put as 2nd parameter
    }

    public void brake() {
        motor.set(ControlMode.DUTY_CYCLE, 0);
    }
}