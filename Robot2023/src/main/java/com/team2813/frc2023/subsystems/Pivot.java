package com.team2813.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team2813.lib.motors.ControlMode;
import com.team2813.lib.motors.TalonFXWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.team2813.frc2023.Constants.*;

public class Pivot extends Subsystem1d<Pivot.Rotations> {

    private boolean positionSet = false;

    public Pivot() {
        super(new TalonFXWrapper(PIVOT_MOTOR_ID, TalonFXInvertType.Clockwise));

        motor.configPID(0.5625, 0, 0);
        motor.configMotionMagic(21000, 21000);

        motor.setEncoderPosition(Rotations.STARTING_CONFIGURATION.position);

        // For finding starting config position from zero
//        motor.setEncoderPosition(0);
//        ((TalonFXWrapper) motor).setNeutralMode(NeutralMode.Coast);
    }

    public double getMotorPosition() {
        return motor.getEncoderPosition();
    }

    public double getMotorVelocity() {
        return motor.getVelocity();
    }

    public double getGoalPosition() {
        return currentEncoderRotationSetpoint;
    }

    public boolean isPositionSet() {
        return positionSet;
    }

    public boolean positionReached() {
        return Math.abs(currentPosition.getPos() - motor.getEncoderPosition()) < 0.05;
    }

    public boolean atZero() {
        return ((TalonFXWrapper) motor).getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    public void startZeroingPivot() {
        motor.set(ControlMode.DUTY_CYCLE, -0.75);
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

    public enum Rotations implements Position {
        STARTING_CONFIGURATION(181),
        HIGH_CONE(122),
        HIGH_CUBE(111),
        MID_CONE(122),
        MID_CUBE(100),
        SINGLE_SUBSTATION(49),
        DOUBLE_SUBSTATION(120);

        @Override
        public double getPos() {
            return position;
        }

        final double position;

        Rotations(double position ) {
            this.position = position;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", getMotorPosition());
        SmartDashboard.putNumber("Pivot Motor Supply Current (A)", ((TalonFXWrapper) motor).getSupplyCurrent());
        SmartDashboard.putNumber("Pivot Motor Stator Current (A)", ((TalonFXWrapper) motor).getStatorCurrent());
    }
}
