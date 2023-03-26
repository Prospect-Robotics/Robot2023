package com.team2813.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team2813.lib.motors.ControlMode;
import com.team2813.lib.motors.TalonFXWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.team2813.frc2023.Constants.*;

public class Arm extends Subsystem1d<Arm.ExtensionLength> {

    private boolean positionSet = false;

    public Arm() {
        super(new TalonFXWrapper(ARM_MOTOR_ID, TalonFXInvertType.Clockwise));

        motor.configPID(0.5, 0, 0);
        motor.configMotionMagic(21000, 20000);
    }

    public double getGoalPosition() {
        return currentEncoderRotationSetpoint;
    }

    public double getMotorPosition() {
        return motor.getEncoderPosition();
    }

    public double getMotorVelocity(){
        return motor.getVelocity();
    }

    public boolean isPositionSet() {
        return positionSet;
    }

    public boolean positionReached() {
        return Math.abs(currentPosition.getPos() - motor.getEncoderPosition()) < 0.5;
    }

    public void startRetractingArm() {
        motor.set(ControlMode.DUTY_CYCLE, -0.5);
        positionSet = false;
    }

    public void brake() {
        motor.set(ControlMode.DUTY_CYCLE, 0);
    }

    @Override
    public void setPosition(double encoderRotations) {
        super.setPosition(encoderRotations);
        positionSet = true;
    }

    public enum ExtensionLength implements Subsystem1d.Position {
        TOP(44),
        MIDDLE(13),
        SINGLE_SUBSTATION(5),
        DOUBLE_SUBSTATION(53);

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
