package com.team2813.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2813.lib.motors.ControlMode;
import com.team2813.lib.motors.SparkMaxWrapper;
import com.team2813.lib.motors.TalonFXWrapper;
import com.team2813.lib.util.ConfigUtils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.team2813.frc2023.Constants.*;

public class Wrist extends Subsystem1d<Wrist.Rotations> {

    public Wrist() {
        super(new TalonFXWrapper(WRIST_MOTOR_ID, TalonFXInvertType.Clockwise));

        motor.configPID(0.5, 0, 0);
        motor.configMotionMagic(12000, 24000);
    }

    public double getMotorPosition() {
        return motor.getEncoderPosition();
    }

    public double getMotorVelocity() {
        return motor.getVelocity();
    }

    public void up() {
        motor.set(ControlMode.DUTY_CYCLE, -0.15);
    }

    public void down() {
        motor.set(ControlMode.DUTY_CYCLE, 0.15);
    }

    public boolean positionReached() {
        return Math.abs(currentPosition.getPos() - motor.getEncoderPosition()) < 1;
    }

    public void startStowingWrist() {
        motor.set(ControlMode.DUTY_CYCLE, -0.55);
    }

    public void brake() {
        motor.set(ControlMode.DUTY_CYCLE, 0);
    }

    public enum Rotations implements Subsystem1d.Position {
        INTAKE(9),
        TOP_SCORE(60),
        MID_SCORE(65),
        DOUBLE_SUBSTATION(25);

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