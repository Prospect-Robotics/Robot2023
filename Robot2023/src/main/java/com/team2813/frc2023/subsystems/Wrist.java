package com.team2813.frc2023.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.team2813.lib.motors.ControlMode;
import com.team2813.lib.motors.SparkMaxWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.team2813.frc2023.Constants.*;

public class Wrist extends Subsystem1d<Wrist.Rotations> {
    public Wrist() {
        super(new SparkMaxWrapper(WRIST_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless, true));

        motor.configPID(0.25, 0, 0);
        motor.configMotionMagic(11000, 10000);
    }

    public double getMotorVelocity() {
        return motor.getVelocity();
    }

    public boolean positionReached() {
        return Math.abs(currentPosition.getPos() - motor.getEncoderPosition()) < 1;
    }

    public void startStowingWrist() {
        motor.set(ControlMode.DUTY_CYCLE, -0.3);
    }

    public void brake() {
        motor.set(ControlMode.DUTY_CYCLE, 0);
    }

    public enum Rotations implements Position {
        INTAKE(9),
        OUTTAKE(20),
        DOUBLE_SUBSTATION(40);

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
        SmartDashboard.putNumber("Wrist Position", motor.getEncoderPosition());
    }
}