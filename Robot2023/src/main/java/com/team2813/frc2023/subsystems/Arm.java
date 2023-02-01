package com.team2813.frc2023.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2813.lib.motors.ControlMode;
import com.team2813.lib.motors.TalonFXProWrapper;

import static com.team2813.frc2023.Constants.*;

public class Arm extends Subsystem1d {

    public Arm() {
        super(new TalonFXProWrapper(ARM_ID, true));

        motor.configPID(0.4, 0, 0); //probably will need to change
        motor.configMotionMagic(30000, 30000); // max vel in ticks/100ms
    }

    public double getMotorVelocity(){
        return motor.getVelocity();
    }

    public boolean positionReached() {
        return Math.abs(currentPosition.getPos() - motor.getEncoderPosition()) < 0.05; //probably change value
    }

    public void startLoweringArm() {
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
