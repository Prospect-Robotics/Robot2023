package com.team2813.frc2023.subsystems;


import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team2813.lib.motors.ControlMode;
import com.team2813.lib.motors.TalonFXWrapper;

import static com.team2813.frc2023.Constants.*;

public class Pivot extends Subsystem1d<Pivot.Rotations> {

    public Pivot() {
        super(new TalonFXWrapper(PIVOT_MOTOR_ID, TalonFXInvertType.Clockwise)); //TODO: Find ot if its inverted or not

        motor.configPID(0, 0, 0);
        motor.configMotionMagic(21000, 20000);

        motor.setEncoderPosition(Rotations.STARTING_CONFIGURATION.position);
    }

    public double getMotorPosition() {
        return motor.getEncoderPosition();
    }

    public double getMotorVelocity(){
        return motor.getVelocity();
    }

    public boolean positionReached() {
        return Math.abs(currentPosition.getPos() - motor.getEncoderPosition()) < 0.05;
    }

    public void startZeroingPivot() {
        motor.set(ControlMode.DUTY_CYCLE, -0.1);
    }

    public void brake() {
        motor.set(ControlMode.DUTY_CYCLE, 0);
    }

    public enum Rotations implements Position {
        STARTING_CONFIGURATION(63),
        HIGH(13),
        MID(6);

        @Override
        public double getPos() {
            return position;
        }

        final double position;

        Rotations(double position ) {
            this.position = position;
        }
    }


}
