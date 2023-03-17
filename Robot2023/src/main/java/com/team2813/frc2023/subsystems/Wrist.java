package com.team2813.frc2023.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.team2813.lib.motors.ControlMode;
import com.team2813.lib.motors.SparkMaxWrapper;
import com.team2813.lib.util.ConfigUtils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.team2813.frc2023.Constants.*;

public class Wrist extends Subsystem1d<Wrist.Rotations> {

    private boolean positionSet = false;
    private boolean manualControl = false;

    public Wrist() {
        super(new SparkMaxWrapper(WRIST_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless, true));

        motor.configPID(0.25, 0, 0);
        motor.configMotionMagic(11000, 10000);

        SparkMaxWrapper sparkMax = (SparkMaxWrapper) motor;
        ConfigUtils.revConfig(() -> sparkMax.setSmartCurrentLimit(30));
    }

    public double getMotorPosition() {
        return motor.getEncoderPosition();
    }

    public double getMotorVelocity() {
        return motor.getVelocity();
    }

    public boolean isPositionSet() {
        return positionSet;
    }

    public boolean isUsingManualControl() {
        return manualControl;
    }

    public void up() {
        motor.set(ControlMode.DUTY_CYCLE, -0.5);
        manualControl = true;
    }

    public void down() {
        motor.set(ControlMode.DUTY_CYCLE, 0.5);
        manualControl = true;
    }

    public void idle() {
        motor.set(ControlMode.DUTY_CYCLE, -0.2);
    }

    public boolean positionReached() {
        return Math.abs(currentPosition.getPos() - motor.getEncoderPosition()) < 1;
    }

    public void startStowingWrist() {
        motor.set(ControlMode.DUTY_CYCLE, -0.3);
        positionSet = false;
        manualControl = false;
    }

    public void brake() {
        motor.set(ControlMode.DUTY_CYCLE, 0);
    }

    public enum Rotations implements Subsystem1d.Position {
        INTAKE(9),
        OUTTAKE(20),
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
    public void setPosition(Rotations position) {
        super.setPosition(position);
        positionSet = true;
        manualControl = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Position", motor.getEncoderPosition());
    }
}