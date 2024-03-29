package com.team2813.lib.motors;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.revrobotics.CANSparkMax.ControlType;

public enum ControlMode {
    DUTY_CYCLE(TalonFXControlMode.PercentOutput, ControlType.kDutyCycle),
    VELOCITY(TalonFXControlMode.Velocity, ControlType.kVelocity),
    MOTION_MAGIC(TalonFXControlMode.MotionMagic, ControlType.kPosition);

    private TalonFXControlMode talonMode;
    private ControlType sparkMode;

    ControlMode(TalonFXControlMode talonMode, ControlType sparkMode) {
        this.talonMode = talonMode;
        this.sparkMode = sparkMode;
    }

    public TalonFXControlMode getTalonMode() {
        return talonMode;
    }

    public ControlType getSparkMode() {
        return sparkMode;
    }
}