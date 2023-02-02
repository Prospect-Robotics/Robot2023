package com.team2813.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team2813.lib.motors.ControlMode;
import com.team2813.lib.motors.TalonFXWrapper;

import static com.team2813.frc2023.Constants.*;

public class Arm extends Subsystem1d<Arm.ExtensionLength> {

    public Arm() {
        super(new TalonFXWrapper(ARM_MOTOR_ID, TalonFXInvertType.Clockwise));

        motor.configPID(0.4, 0, 0); // TODO: tune
        motor.configMotionMagic(21000, 20000);
    }

    public double getMotorVelocity(){
        return motor.getVelocity();
    }

    public boolean positionReached() {
        return Math.abs(currentPosition.getPos() - motor.getEncoderPosition()) < 0.05; //probably change value
    }

    public void startRetractingArm() {
        motor.set(ControlMode.DUTY_CYCLE, -0.98); //probably change value
    }

    public void brake() {
        motor.set(ControlMode.DUTY_CYCLE, 0);
    }

    public enum ExtensionLength implements Position {
        INTAKE(26),
        MIDDLE(62.25),
        TOP(136.6),
        DOUBLE_SUBSTATION(80);

        @Override
        public double getPos() {
            return position;
        }

        @Override
        public Position getMin() {
            return INTAKE;
        }

        @Override
        public Position getMax() {
            return TOP;
        }

        final double position;

        ExtensionLength(double position ) {
            this.position = position;
        }
    }
}
