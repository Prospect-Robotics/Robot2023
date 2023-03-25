package com.team2813.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team2813.lib.motors.ControlMode;
import com.team2813.lib.motors.TalonFXWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.team2813.frc2023.Constants.*;

public class Pivot extends Subsystem1d<Pivot.Rotations> {

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

    public double getMotorVelocity(){
        return motor.getVelocity();
    }

    public boolean positionReached() {
        return Math.abs(currentPosition.getPos() - motor.getEncoderPosition()) < 0.05;
    }

    public boolean atZero() {
        return ((TalonFXWrapper) motor).getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    public void startZeroingPivot() {
        motor.set(ControlMode.DUTY_CYCLE, -0.75);
    }

    public void brake() {
        motor.set(ControlMode.DUTY_CYCLE, 0);
    }
    
    public enum Rotations implements Position {
        STARTING_CONFIGURATION(189),
        HIGH(125),
        MID(127),
        DOUBLE_SUBSTATION(122),
        SINGLE_SUBSTATION(53);

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
    }
}
