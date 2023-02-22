package com.team2813.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team2813.lib.motors.ControlMode;
import com.team2813.lib.motors.TalonFXWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.team2813.frc2023.Constants.*;

public class Arm extends Subsystem1d<Arm.ExtensionLength> {

    public Arm() {
        super(new TalonFXWrapper(ARM_MOTOR_ID, TalonFXInvertType.Clockwise));

        motor.configPID(0.25, 0, 0); // TODO: tune
        motor.configMotionMagic(21000, 20000);
    }

    public double getMotorPosition() {
        return motor.getEncoderPosition();
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
        INTAKE(2),
        MIDDLE(25),
        TOP(35),
        DOUBLE_SUBSTATION(35);

        @Override
        public double getPos() {
            return position;
        }

        final double position;

        ExtensionLength(double position) {
            this.position = position;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", motor.getEncoderPosition());
    }
}
