package com.team2813.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2813.lib.motors.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Subsystem1d<P extends Subsystem1d.Position> extends SubsystemBase {

    protected Motor motor;
    protected TalonFXProWrapper talonFXPro;
    protected Position currentPosition;
    protected double goalRotations;

    public Subsystem1d(SparkMaxWrapper motor) {
        this.motor = motor;
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 25);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100);
        motor.set(ControlMode.DUTY_CYCLE, 0);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public Subsystem1d(TalonFXWrapper motor) {
        this.motor = motor;

        if (!motor.isOnCANivore()) {
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 125);
        }

        motor.set(ControlMode.DUTY_CYCLE, 0);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public Subsystem1d(TalonFXProWrapper motor) {
        talonFXPro = motor;

        if (!motor.isOnCANivore()) {
            motor.configPositionSignalUpdateFrequency(4);
            motor.configVelocitySignalUpdateFrequency(4);
        }

        motor.set(ControlMode.DUTY_CYCLE, 0);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void zeroSensors() {
        if (talonFXPro != null) talonFXPro.setEncoderPosition(0);
        else motor.setEncoderPosition(0);
    }

    /*==========================
     * POSITION
     * ==========================*/

    protected interface Position {
        /**
         * @return encoder rotations of given position
         */
        double getPos();
    }

    public void setPosition(double encoderRotations) {
        goalRotations = encoderRotations;

        if (talonFXPro != null) talonFXPro.set(ControlMode.MOTION_MAGIC, encoderRotations);
        else motor.set(ControlMode.MOTION_MAGIC, encoderRotations);
    }

    public void setPosition(P position) {
        currentPosition = position;
        setPosition(position.getPos());
    }
}